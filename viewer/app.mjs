// SPDX-License-Identifier: GPL-3.0-or-later
import {TelemetryParser, TraceBuffer, COLUMNS, csvRow} from './telemetry.mjs';

const $=id=>document.getElementById(id), buffer=new TraceBuffer(10000);
const colors=['#2f80ed','#009c86','#c77912'], angleKeys=['roll','pitch','yaw'];
let visible=[true,true,true],paused=false,frozen=[],hoverFraction=null,selected=null,lastView=null;
let source='none',port=null,reader=null,writer=null,connecting=false,demoTimer=null;
let latest=null,lastReceived=0,total=0,rateCount=0,rateStart=performance.now(),frequency=0;
let recording=false,capture=[],toastTimer,dirty=true,lastPaint=0;
const parser=new TelemetryParser(receive,info=>{
  if(info.includes('ERROR'))toast(info);
  if(info.includes('IMU Library'))$('connectionHint').textContent=info;
});

function toast(message){$('toast').textContent=message;$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),4500);}
function controls(){
  $('connect').disabled=connecting;$('connect').textContent=port?'Ngắt USB':connecting?'Đang kết nối…':'↔ Kết nối USB';
  $('baud').disabled=Boolean(port)||connecting;$('calibrate').disabled=$('zero').disabled=!writer;
  $('record').disabled=!(source==='demo'||source==='serial'&&port)||connecting;
  $('import').disabled=$('demo').disabled=Boolean(port)||connecting;
}
function setSource(value){
  source=value;$('sourceBadge').className='badge'+(value==='demo'?' demo':value==='serial'?' live':'');
  $('sourceBadge').textContent=({none:'Chưa kết nối',demo:'MÔ PHỎNG',serial:'USB',csv:'CSV'})[value];controls();dirty=true;
}
function clearData(){
  buffer.clear();latest=null;selected=null;hoverFraction=null;lastView=null;total=rateCount=0;frequency=0;rateStart=performance.now();
  paused=false;frozen=[];updatePauseButton();dirty=true;
}
function receive(sample){
  if(latest&&sample.t_us<latest.t_us){hoverFraction=null;selected=null;}
  latest=buffer.add(sample);lastReceived=performance.now();total++;rateCount++;if(!paused)dirty=true;
  if(recording){capture.push(csvRow(sample));if(capture.length>=120000){finishCapture();toast('Đã lưu 120.000 mẫu. Có thể bắt đầu bản ghi mới.');}}
}
function stopDemo(){if(demoTimer!==null){clearInterval(demoTimer);demoTimer=null;}}
function startDemo(){
  if(port)return;if(recording)finishCapture();stopDemo();clearData();parser.reset();setSource('demo');
  const start=performance.now();
  demoTimer=setInterval(()=>{
    const t=(performance.now()-start)/1000;
    receive({t_us:Math.round(t*1e6),roll:12*Math.sin(t*.75),pitch:7*Math.sin(t*.5+.7),yaw:(t*3.5)%360,
      gx:.157*Math.cos(t*.75),gy:.061*Math.cos(t*.5+.7),gz:.061,ax:.4*Math.sin(t*.8),ay:1.4*Math.sin(t*.75),az:9.8+.12*Math.sin(t),
      temp:27+.5*Math.sin(t/20),ready:t>2?1:0,dt_us:1000,gaps:0,calib:Math.min(1,t/2),retries:0,late:0,roll_sign:1});
  },10);controls();
}
async function disconnect(){if(reader){try{await reader.cancel();}catch{}}else if(port){try{await port.close();}catch{}port=null;}controls();dirty=true;}
async function connect(){
  if(port){await disconnect();return;}
  if(!('serial' in navigator)){toast('USB cần Chrome hoặc Edge trên máy tính.');return;}
  stopDemo();if(recording)finishCapture();connecting=true;controls();let requested;
  try{
    requested=await navigator.serial.requestPort();await requested.open({baudRate:Number($('baud').value),bufferSize:16384});
    port=requested;writer=port.writable.getWriter();reader=port.readable.getReader();
    parser.reset();clearData();setSource('serial');const decoder=new TextDecoder();connecting=false;controls();
    try{while(true){const {value,done}=await reader.read();if(done)break;if(value)parser.push(decoder.decode(value,{stream:true}));}}
    finally{reader.releaseLock();reader=null;}
  }catch(error){if(error.name!=='NotFoundError')toast(`Không thể kết nối: ${error.message}`);}
  finally{
    if(writer){try{writer.releaseLock();}catch{}writer=null;}if(requested){try{await requested.close();}catch{}}
    port=null;connecting=false;if(recording)finishCapture();controls();dirty=true;
  }
}
async function command(text){
  if(!writer)return;try{await writer.write(new TextEncoder().encode(text+'\n'));
    if(text==='CALIBRATE')toast('Đã gửi hiệu chuẩn. Giữ bo đứng yên.');
    else{clearData();toast('Đã đặt mốc yaw.');}
  }catch(error){toast('Không gửi được lệnh: '+error.message);}
}
function download(text,name){const url=URL.createObjectURL(new Blob([text],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
function finishCapture(){
  recording=false;$('record').classList.remove('recording');$('record').textContent='● Ghi CSV';
  if(capture.length){const name=`ttimu-${source==='demo'?'SIMULATED-':''}${new Date().toISOString().replace(/[:.]/g,'-')}.csv`;
    download(`# IMU Library by TT; source=${source}; ${source==='demo'?'SIMULATED DATA':'recorded telemetry'}; values=as-received\n${COLUMNS.join(',')}\n${capture.join('\n')}\n`,name);}
  $('recordInfo').textContent=`Đã lưu ${capture.length.toLocaleString('vi-VN')} mẫu`;capture=[];
}
async function importFile(file){
  if(!file)return;if(file.size>64*1024*1024){toast('Chọn CSV nhỏ hơn 64 MB.');return;}
  stopDemo();if(recording)finishCapture();clearData();parser.reset();setSource('csv');$('sourceBadge').title=file.name;
  try{parser.push(await file.text());parser.finish();const points=buffer.values();
    if(points.length>1){const span=(points.at(-1).t_us-points[0].t_us)/1e6;frequency=span>0?(points.length-1)/span:0;}
    $('window').value='all';dirty=true;if(parser.errors)toast(`Bỏ ${parser.errors} dòng lỗi.`);
  }catch(error){toast('Không đọc được CSV: '+error.message);}
}

function renderLegend(){
  $('legend').replaceChildren();angleKeys.forEach((key,i)=>{
    const b=document.createElement('button');b.style.setProperty('--channel',colors[i]);b.setAttribute('aria-pressed','true');
    b.setAttribute('aria-label',`Hiện hoặc ẩn ${key}`);b.title='Góc liên tục, cùng giá trị trên đồ thị';
    const dot=document.createElement('i'),label=document.createElement('span'),value=document.createElement('strong');
    label.textContent=key[0].toUpperCase()+key.slice(1);label.prepend(dot);value.id=key;value.textContent='—';b.append(label,value);
    b.onclick=()=>{visible[i]=!visible[i];b.classList.toggle('hidden',!visible[i]);b.setAttribute('aria-pressed',String(visible[i]));dirty=true;};$('legend').append(b);
  });
}
const format=(v,digits=1)=>Number.isFinite(v)?v.toFixed(digits):'—';
function valueFor(p,key){
  if(!p)return undefined;
  if(key==='roll')return p.roll_unwrapped??p.roll;
  if(key==='yaw')return p.yaw_unwrapped??p.yaw;
  return p[key];
}
function updateReadings(){
  const point=selected??(paused?frozen.at(-1):latest);
  for(const key of angleKeys)$(key).textContent=format(valueFor(point,key),2);
  for(const key of ['gx','gy','gz','ax','ay','az'])$(key).textContent=format(point?.[key],key[0]==='g'?5:3);
  $('temperature').textContent=format(latest?.temp);$('frequency').textContent=latest?format(frequency,0):'—';
  $('dt').textContent=format(latest?.dt_us,0);$('gaps').textContent=format(latest?.gaps,0);
  const progress=Number.isFinite(latest?.calib)?Math.max(0,Math.min(1,latest.calib)):latest?.ready===1?1:0;
  $('calibProgress').style.width=`${progress*100}%`;$('calibPercent').textContent=latest?`${Math.round(progress*100)}%`:'—';
  $('calibLabel').textContent=latest?.ready===1?'Đã hiệu chuẩn':latest?.ready===0?'Đang hiệu chuẩn':'Hiệu chuẩn';
  $('sampleCount').textContent=`${total.toLocaleString('vi-VN')} mẫu${parser.errors?` · ${parser.errors} lỗi`:''}`;
  const stale=source==='serial'&&(!port||performance.now()-lastReceived>1500),ready=latest?.ready===1;
  let message='Chưa có dữ liệu';
  if(source==='demo')message='MÔ PHỎNG';
  else if(source==='csv')message=latest?'CSV · đã ghi':'CSV không có mẫu hợp lệ';
  else if(source==='serial')message=!port?'USB đã ngắt':stale?'Chờ dữ liệu USB':!ready?'Đang hiệu chuẩn':latest.gaps>0?'Có khoảng mất mẫu':'Dữ liệu sẵn sàng';
  $('noticeText').textContent=message;$('notice').className='signal'+(ready&&source==='serial'&&!stale?' ready':source==='demo'||stale||latest?.gaps>0?' warn':'');
  $('streamLabel').textContent=paused?'TẠM DỪNG HÌNH':source==='demo'?'MÔ PHỎNG':source==='csv'?'CSV':port&&!stale?'USB STREAM':'CHƯA KẾT NỐI';
  $('liveDot').className=port&&!stale?'live':'';
  $('cursorLabel').textContent=selected?`t = ${(selected.t_us/1e6).toFixed(3)} s`:'';
  if(recording)$('recordInfo').textContent=`${capture.length.toLocaleString('vi-VN')} mẫu đang ghi`;
  const title=`${message}${latest&&Number.isFinite(latest.temp)?` · ${format(latest.temp)} °C`:''} | IMU by TT`;
  if(document.title!==title)document.title=title;controls();
}

const plots=[{canvas:$('plot'),keys:angleKeys,colors,large:true},...['gx','gy','gz','ax','ay','az'].map((key,i)=>({canvas:$('plot-'+key),keys:[key],colors:[colors[i%3]],large:false}))];
function makeView(){
  const all=paused?frozen:buffer.values(),end=all.length?all.at(-1).t_us/1e6:0;
  const seconds=$('window').value==='all'?Math.max(.1,end-(all[0]?.t_us??0)/1e6):Number($('window').value),start=end-seconds;
  const points=all.filter(p=>p.t_us/1e6>=start),intervals=[];
  for(let i=1;i<points.length&&i<=128;i++)intervals.push(points[i].t_us-points[i-1].t_us);
  intervals.sort((a,b)=>a-b);
  return {all,end,seconds,start,points,gapUs:Math.max(250000,5*(intervals[intervals.length>>1]||0))};
}
function nearest(points,time){
  if(!points.length)return null;let lo=0,hi=points.length-1;
  while(lo<hi){const mid=(lo+hi)>>1;if(points[mid].t_us<time)lo=mid+1;else hi=mid;}
  return lo&&Math.abs(points[lo-1].t_us-time)<Math.abs(points[lo].t_us-time)?points[lo-1]:points[lo];
}
function drawChart(plot,view){
  const canvas=plot.canvas,rect=canvas.getBoundingClientRect();if(rect.width<1||rect.height<1)return;
  const dpr=Math.min(window.devicePixelRatio||1,2),ctx=canvas.getContext('2d');
  if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);}
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);ctx.fillStyle='#fff';ctx.fillRect(0,0,rect.width,rect.height);
  const left=plot.large?57:43,right=plot.large?13:7,top=plot.large?13:6,bottom=plot.large?26:18,w=rect.width-left-right,h=rect.height-top-bottom;
  if(w<=0||h<=0)return;
  const enabled=i=>!plot.large||visible[i];let min=Infinity,max=-Infinity;
  for(const p of view.points)for(let i=0;i<plot.keys.length;i++)if(enabled(i)){const v=valueFor(p,plot.keys[i]);if(Number.isFinite(v)){min=Math.min(min,v);max=Math.max(max,v);}}
  const floor=plot.large?2:plot.keys[0][0]==='g'?.004:.04;
  if(!Number.isFinite(min)){min=-floor/2;max=floor/2;}
  const span=Math.max(max-min,floor),middle=(min+max)/2;min=middle-span*.6;max=middle+span*.6;
  const yOf=v=>top+(max-v)/(max-min)*h,xOf=p=>left+(p.t_us/1e6-view.start)/view.seconds*w;
  ctx.font=`${plot.large?10:9}px Consolas,monospace`;ctx.textBaseline='middle';ctx.lineWidth=1;
  const rows=plot.large?4:2;
  for(let i=0;i<=rows;i++){
    const y=top+h*i/rows,v=max-(max-min)*i/rows;ctx.strokeStyle='#ddd';ctx.beginPath();ctx.moveTo(left,y+.5);ctx.lineTo(left+w,y+.5);ctx.stroke();
    const digits=span<.1?3:span<2?2:span<20?1:0;ctx.fillStyle='#111';ctx.textAlign='right';ctx.fillText(Math.abs(v)>99999?v.toExponential(1):v.toFixed(digits),left-7,y);
  }
  const cols=plot.large?5:2;
  for(let i=0;i<=cols;i++){
    const x=left+w*i/cols;ctx.strokeStyle='#eee';ctx.beginPath();ctx.moveTo(x+.5,top);ctx.lineTo(x+.5,top+h);ctx.stroke();
    ctx.fillStyle='#111';ctx.textAlign='center';const tick=-view.seconds+view.seconds*i/cols;
    ctx.fillText(`${(Math.abs(tick)<.0001?0:tick).toFixed(view.seconds/cols<1?1:0)}s`,x,top+h+12);
  }
  ctx.save();ctx.beginPath();ctx.rect(left,top,w,h);ctx.clip();
  for(let i=0;i<plot.keys.length;i++){
    if(!enabled(i))continue;ctx.strokeStyle=plot.colors[i];ctx.lineWidth=plot.large?1.6:1.2;ctx.beginPath();let previous=null;
    for(const p of view.points){const v=valueFor(p,plot.keys[i]);if(!Number.isFinite(v)){previous=null;continue;}
      const x=xOf(p),y=yOf(v);if(previous&&p.t_us-previous.t_us<view.gapUs)ctx.lineTo(x,y);else ctx.moveTo(x,y);previous=p;
    }ctx.stroke();
  }
  if(selected){const x=xOf(selected);ctx.strokeStyle='#1118';ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,top+h);ctx.stroke();ctx.setLineDash([]);
    for(let i=0;i<plot.keys.length;i++){const v=valueFor(selected,plot.keys[i]);if(enabled(i)&&Number.isFinite(v)){ctx.fillStyle=plot.colors[i];ctx.beginPath();ctx.arc(x,yOf(v),plot.large?3:2.5,0,Math.PI*2);ctx.fill();}}
  }ctx.restore();
}
function draw(){lastView=makeView();selected=hoverFraction===null?null:nearest(lastView.points,(lastView.start+hoverFraction*lastView.seconds)*1e6);for(const p of plots)drawChart(p,lastView);$('empty').hidden=Boolean(lastView.all.length);}
function updatePauseButton(){$('pause').textContent=paused?'▷':'Ⅱ';$('pause').classList.toggle('active',paused);$('pause').setAttribute('aria-label',paused?'Tiếp tục đồ thị':'Tạm dừng đồ thị');}
$('connect').onclick=connect;$('demo').onclick=$('emptyDemo').onclick=startDemo;
$('calibrate').onclick=()=>command('CALIBRATE');$('zero').onclick=()=>command('ZERO_YAW');
$('import').onclick=()=>$('file').click();$('file').onchange=e=>{importFile(e.target.files[0]);e.target.value='';};
$('clear').onclick=clearData;$('window').onchange=()=>{dirty=true;};
$('pause').onclick=()=>{paused=!paused;frozen=paused?buffer.values():[];hoverFraction=null;selected=null;updatePauseButton();dirty=true;};
$('record').onclick=()=>{if(recording){finishCapture();return;}if(!(source==='demo'||source==='serial'&&port))return;recording=true;capture=[];$('record').textContent='■ Lưu CSV';$('record').classList.add('recording');};
const resize=new ResizeObserver(()=>{dirty=true;});
for(const plot of plots){resize.observe(plot.canvas);plot.canvas.addEventListener('pointermove',e=>{
  if(e.pointerType==='touch')return;const r=plot.canvas.getBoundingClientRect(),left=plot.large?57:43,right=plot.large?13:7;
  hoverFraction=Math.max(0,Math.min(1,(e.clientX-r.left-left)/(r.width-left-right)));dirty=true;
});plot.canvas.addEventListener('pointerleave',()=>{hoverFraction=null;selected=null;dirty=true;});}
function frame(now){
  if(now-rateStart>=1000&&source!=='csv'){frequency=rateCount*1000/(now-rateStart);rateCount=0;rateStart=now;}
  if(now-lastPaint>=50){if(dirty){draw();dirty=false;}updateReadings();lastPaint=now;}
  requestAnimationFrame(frame);
}
if(!('serial' in navigator))$('connectionHint').textContent='USB cần Chrome / Edge trên máy tính.';
window.addEventListener('beforeunload',e=>{if(recording&&capture.length){e.preventDefault();e.returnValue='';}});
renderLegend();controls();requestAnimationFrame(frame);
