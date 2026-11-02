// SPDX-License-Identifier: GPL-3.0-or-later
import {TelemetryParser, TraceBuffer, COLUMNS, csvRow} from './telemetry.mjs';

const $ = id => document.getElementById(id);
const buffer = new TraceBuffer(10000);
const colors = ['#69a5ff','#36d2ad','#f0b959'];
const modes = {
  attitude: {keys:['roll','pitch','yaw_unwrapped'], labels:['Roll','Pitch','Yaw'], unit:'Góc (°) · yaw nối liên tục qua 360°'},
  gyro: {keys:['gx','gy','gz'], labels:['X','Y','Z'], unit:'Vận tốc góc (rad/s)'},
  accel: {keys:['ax','ay','az'], labels:['X','Y','Z'], unit:'Gia tốc gồm trọng lực (m/s²)'}
};
let mode='attitude', visible=[true,true,true], paused=false, source='none';
let port=null, reader=null, writer=null, connecting=false, demoTimer=null;
let latest=null, lastReceived=0, total=0, rateCount=0, rateStart=performance.now(), frequency=0;
let recording=false, capture=[], toastTimer, dirty=true, lastPaint=0;
const parser=new TelemetryParser(receive, info => {
  if(info.includes('ERROR'))toast(info);
  if(info.includes('IMU Library'))$('connectionHint').textContent=info;
});

function toast(message) {
  $('toast').textContent=message;$('toast').classList.add('visible');
  clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),5500);
}
function setSource(value) {
  source=value;
  $('sourceBadge').className='badge'+(value==='demo'?' demo':value==='serial'?' live':'');
  $('sourceBadge').textContent=({none:'Chưa có dữ liệu',demo:'MÔ PHỎNG · không phải IMU thật',serial:'USB · dữ liệu từ bo',csv:'CSV · dữ liệu đã ghi'})[value];
  controls();dirty=true;
}
function controls() {
  $('connect').disabled=connecting;
  $('connect').textContent=port?'Ngắt kết nối USB':connecting?'Đang kết nối…':'↔ Kết nối USB';
  $('baud').disabled=Boolean(port)||connecting;
  $('calibrate').disabled=$('zero').disabled=!writer;
  $('record').disabled=!latest||source==='csv';
  $('import').disabled=Boolean(port)||connecting;
  $('demo').disabled=Boolean(port)||connecting;
}
function clearData() {
  buffer.clear();latest=null;total=rateCount=0;frequency=0;rateStart=performance.now();
  dirty=true;updateReadings();controls();
}
function receive(sample) {
  latest=buffer.add(sample);lastReceived=performance.now();total++;rateCount++;dirty=true;
  if(recording) {
    capture.push(csvRow(sample));
    if(capture.length>=120000){finishCapture();toast('Đã lưu CSV khi đạt giới hạn 120.000 mẫu. Bạn có thể bắt đầu bản ghi mới.');}
  }
}
function stopDemo() {if(demoTimer!==null){clearInterval(demoTimer);demoTimer=null;}}
function startDemo() {
  if(port)return;
  if(recording)finishCapture();stopDemo();clearData();parser.reset();setSource('demo');
  const start=performance.now();
  demoTimer=setInterval(()=>{
    const t=(performance.now()-start)/1000;
    receive({t_us:Math.round(t*1e6),roll:12*Math.sin(t*.75),pitch:7*Math.sin(t*.5+.7),yaw:(t*3.5)%360,
      gx:.157*Math.cos(t*.75),gy:.061*Math.cos(t*.5+.7),gz:.061,
      ax:.4*Math.sin(t*.8),ay:1.4*Math.sin(t*.75),az:9.8+.12*Math.sin(t),
      temp:27+.5*Math.sin(t/20),ready:t>2?1:0,dt_us:1000,gaps:0,calib:Math.min(1,t/2),retries:0,late:0});
  },10);
  controls();toast('Đang xem dữ liệu mô phỏng để thử giao diện. Chưa kết nối bo thật.');
}

async function disconnect() {
  if(reader){try{await reader.cancel();}catch{}}
  else if(port){try{await port.close();}catch{}port=null;}
  controls();dirty=true;
}
async function connect() {
  if(port){await disconnect();return;}
  if(!('serial' in navigator)){
    toast('Web Serial cần Chrome hoặc Edge trên máy tính. Bạn vẫn có thể mở CSV và thử giao diện.');return;
  }
  stopDemo();if(recording)finishCapture();connecting=true;controls();
  let requested;
  try {
    requested=await navigator.serial.requestPort();
    await requested.open({baudRate:Number($('baud').value),bufferSize:16384});
    port=requested;writer=port.writable.getWriter();reader=port.readable.getReader();
    parser.reset();clearData();setSource('serial');
    const decoder=new TextDecoder();
    connecting=false;controls();
    try {
      while(true){const {value,done}=await reader.read();if(done)break;if(value)parser.push(decoder.decode(value,{stream:true}));}
    } finally {reader.releaseLock();reader=null;}
  } catch(error) {
    if(error.name!=='NotFoundError')toast(`Không thể kết nối: ${error.message}`);
  } finally {
    if(writer){try{writer.releaseLock();}catch{}writer=null;}
    if(requested){try{await requested.close();}catch{}}
    port=null;connecting=false;
    if(recording)finishCapture();controls();dirty=true;
  }
}
async function command(text) {
  if(!writer)return;
  try {
    await writer.write(new TextEncoder().encode(text+'\n'));
    if(text==='CALIBRATE')toast('Đã gửi yêu cầu hiệu chuẩn. Giữ bo đứng yên và theo dõi trạng thái từ bo.');
    else{buffer.clear();dirty=true;toast('Đã gửi yêu cầu đặt mốc yaw.');}
  }catch(error){toast('Không gửi được lệnh: '+error.message);}
}
function download(text,name) {
  const url=URL.createObjectURL(new Blob([text],{type:'text/csv;charset=utf-8'}));
  const anchor=document.createElement('a');anchor.href=url;anchor.download=name;anchor.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function finishCapture() {
  recording=false;$('record').classList.remove('recording');$('record').textContent='● Bắt đầu ghi CSV';
  if(capture.length){
    const name=`ttimu-${source==='demo'?'SIMULATED-':''}${new Date().toISOString().replace(/[:.]/g,'-')}.csv`;
    download(`# IMU Library by TT; source=${source}; ${source==='demo'?'SIMULATED DATA':'recorded telemetry'}\n${COLUMNS.join(',')}\n${capture.join('\n')}\n`,name);
  }
  $('recordInfo').textContent=`Đã ghi ${capture.length.toLocaleString('vi-VN')} mẫu.`;capture=[];
}
async function importFile(file) {
  if(!file)return;
  if(file.size>25*1024*1024){toast('Chọn CSV nhỏ hơn 25 MB để trình duyệt xử lý ổn định.');return;}
  stopDemo();if(recording)finishCapture();clearData();parser.reset();setSource('csv');
  try {
    parser.push(await file.text());parser.finish();
    const points=buffer.values();
    if(points.length>1){const span=(points.at(-1).t_us-points[0].t_us)/1e6;frequency=span>0?(points.length-1)/span:0;}
    toast(`${file.name}: ${total.toLocaleString('vi-VN')} mẫu hợp lệ${parser.errors?`; bỏ ${parser.errors} dòng không hợp lệ`:''}.`);
    controls();dirty=true;
  }catch(error){toast('Không đọc được CSV: '+error.message);}
}

function renderLegend() {
  $('legend').replaceChildren();
  modes[mode].labels.forEach((label,i)=>{
    const button=document.createElement('button');button.className=visible[i]?'':'hidden';
    button.setAttribute('aria-pressed',String(visible[i]));button.setAttribute('aria-label',`Hiện hoặc ẩn ${label}`);
    const dot=document.createElement('i');dot.style.background=colors[i];button.append(dot,document.createTextNode(label));
    button.onclick=()=>{visible[i]=!visible[i];renderLegend();dirty=true;};$('legend').append(button);
  });
  $('chartUnit').textContent=modes[mode].unit;
}
function format(value,digits=1) {return Number.isFinite(value)?value.toFixed(digits):'—';}
function updateReadings() {
  for(const key of ['roll','pitch','yaw'])$(key).textContent=latest?format(latest[key],2):'—';
  $('temperature').innerHTML=`${format(latest?.temp)} <em>°C</em>`;
  $('frequency').innerHTML=`${latest?format(frequency,0):'—'} <em>Hz</em>`;
  $('dt').innerHTML=`${format(latest?.dt_us,0)} <em>µs</em>`;
  $('gaps').textContent=latest?format(latest.gaps,0):'—';
  const progress=latest&&Number.isFinite(latest.calib)?Math.max(0,Math.min(1,latest.calib)):0;
  $('calibProgress').style.width=`${progress*100}%`;$('calibPercent').textContent=latest?`${Math.round(progress*100)}%`:'—';
  $('calibLabel').textContent=latest?.ready===1?'Đã hiệu chuẩn':latest?'Đang hiệu chuẩn':'Hiệu chuẩn';
  $('sampleCount').textContent=`${total.toLocaleString('vi-VN')} mẫu${parser.errors?` · ${parser.errors} dòng lỗi`:''}`;
  const stale=source==='serial'&&(!port||performance.now()-lastReceived>1500);
  const ready=latest?.ready===1;
  let message='Sẵn sàng kết nối. Hoặc mở CSV để xem dữ liệu đã ghi.';
  if(source==='demo')message='DỮ LIỆU MÔ PHỎNG — chỉ để thử giao diện, không phải kết quả từ bo IMU.';
  else if(source==='csv')message=latest?'Đang xem CSV. Đồ thị hiển thị tối đa 10.000 mẫu cuối của phiên.':'CSV chưa có mẫu hợp lệ. Cần các cột t_us, roll, pitch, yaw.';
  else if(source==='serial'){
    if(!port)message='USB đã ngắt. Dữ liệu cuối được giữ lại để bạn xem.';
    else if(stale)message='Đã mở USB, đang chờ dữ liệu. Kiểm tra baudrate và nạp ví dụ SerialPlotter.';
    else if(!ready)message=`Đang hiệu chuẩn — giữ bo đứng yên${latest?.retries?`. Đã thử lại ${latest.retries} lần`:''}.`;
    else if(latest.gaps>0)message=`Đang nhận dữ liệu. Bo đã ghi nhận ${latest.gaps} khoảng mất mẫu; kiểm tra tải xử lý và thời gian đọc.`;
    else message='Dữ liệu sẵn sàng. Bạn có thể di chuyển bo hoặc bắt đầu ghi CSV.';
  }
  $('noticeText').textContent=message;
  $('notice').className='notice'+(ready&&source==='serial'&&!stale?' ready':source==='demo'||stale||latest?.gaps>0?' warn':'');
  $('streamLabel').textContent=paused?'ĐỒ THỊ TẠM DỪNG':source==='demo'?'MÔ PHỎNG':source==='csv'?'DỮ LIỆU CSV':port&&!stale?'USB STREAM':'CHƯA KẾT NỐI';
  $('liveDot').className=port&&!stale?'live':'';
  if(recording)$('recordInfo').textContent=`Đang ghi ${capture.length.toLocaleString('vi-VN')} mẫu…`;
  controls();
}

const canvas=$('plot'),ctx=canvas.getContext('2d');
function draw() {
  const rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return;
  const dpr=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);}
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);
  const left=53,right=14,top=16,bottom=31,w=rect.width-left-right,h=rect.height-top-bottom;
  const all=buffer.values(),end=all.length?all.at(-1).t_us/1e6:0,seconds=Number($('window').value),start=end-seconds;
  const points=all.filter(p=>p.t_us/1e6>=start),keys=modes[mode].keys;
  let min=Infinity,max=-Infinity;
  for(const p of points)for(let c=0;c<3;c++)if(visible[c]&&Number.isFinite(p[keys[c]])){min=Math.min(min,p[keys[c]]);max=Math.max(max,p[keys[c]]);}
  if(!Number.isFinite(min)){min=mode==='attitude'?-90:-1;max=mode==='attitude'?90:1;}
  const span=Math.max(max-min,mode==='attitude'?2:.2),middle=(min+max)/2;
  min=middle-span*.65;max=middle+span*.65;
  ctx.font='10px Consolas,monospace';ctx.lineWidth=1;ctx.textBaseline='middle';
  for(let i=0;i<=4;i++){
    const y=top+h*i/4;ctx.strokeStyle='#213147';ctx.beginPath();ctx.moveTo(left,y+.5);ctx.lineTo(left+w,y+.5);ctx.stroke();
    const v=max-(max-min)*i/4;ctx.fillStyle='#647f9e';ctx.textAlign='right';
    ctx.fillText(Math.abs(v)>10000?v.toExponential(1):v.toFixed(Math.abs(max-min)<5?1:0),left-10,y);
  }
  for(let i=0;i<=5;i++){
    const x=left+w*i/5;ctx.strokeStyle='#1b293d';ctx.beginPath();ctx.moveTo(x+.5,top);ctx.lineTo(x+.5,top+h);ctx.stroke();
    ctx.fillStyle='#5c7595';ctx.textAlign='center';ctx.fillText(`${Math.round(-seconds+seconds*i/5)}s`,x,top+h+18);
  }
  ctx.save();ctx.beginPath();ctx.rect(left,top,w,h);ctx.clip();
  for(let c=0;c<3;c++){
    if(!visible[c])continue;ctx.strokeStyle=colors[c];ctx.lineWidth=1.6;ctx.beginPath();let started=false;
    for(const p of points){const v=p[keys[c]];if(!Number.isFinite(v)){started=false;continue;}
      const x=left+(p.t_us/1e6-start)/seconds*w,y=top+(max-v)/(max-min)*h;
      if(started)ctx.lineTo(x,y);else{ctx.moveTo(x,y);started=true;}
    }ctx.stroke();
  }ctx.restore();$('empty').hidden=Boolean(all.length);
}

$('connect').onclick=connect;$('demo').onclick=$('emptyDemo').onclick=startDemo;
$('calibrate').onclick=()=>command('CALIBRATE');$('zero').onclick=()=>command('ZERO_YAW');
$('import').onclick=()=>$('file').click();$('file').onchange=event=>{importFile(event.target.files[0]);event.target.value='';};
$('clear').onclick=clearData;$('window').onchange=()=>{dirty=true;};
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'▷':'Ⅱ';$('pause').setAttribute('aria-label',paused?'Tiếp tục đồ thị':'Tạm dừng đồ thị');dirty=true;};
$('record').onclick=()=>{
  if(recording){finishCapture();return;}
  if(!latest||source==='csv')return;recording=true;capture=[];$('record').textContent='■ Dừng và tải CSV';$('record').classList.add('recording');
};
document.querySelectorAll('[data-mode]').forEach(button=>button.onclick=()=>{
  mode=button.dataset.mode;document.querySelectorAll('[data-mode]').forEach(b=>{b.classList.toggle('active',b===button);b.setAttribute('aria-selected',String(b===button));});visible=[true,true,true];renderLegend();dirty=true;
});
new ResizeObserver(()=>{dirty=true;}).observe(canvas);
function frame(now){
  if(now-rateStart>=1000&&source!=='csv'){frequency=rateCount*1000/(now-rateStart);rateCount=0;rateStart=now;}
  if(now-lastPaint>50){updateReadings();if(dirty&&!paused){draw();dirty=false;}lastPaint=now;}
  requestAnimationFrame(frame);
}
if(!('serial' in navigator))$('connectionHint').textContent='USB cần Chrome / Edge trên máy tính. Bạn vẫn có thể xem CSV hoặc thử giao diện.';
window.addEventListener('beforeunload',event=>{if(recording&&capture.length){event.preventDefault();event.returnValue='';}});
renderLegend();controls();requestAnimationFrame(frame);
