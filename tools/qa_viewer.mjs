// QA uses an isolated headless Chrome profile and explicitly mocked serial data.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
const root=path.resolve('.'),cdp=process.env.TTIMU_QA_CDP_URL||'http://127.0.0.1:9227',url=process.env.TTIMU_QA_VIEWER_URL||'http://127.0.0.1:8093';
const directory=path.join(root,'.cache','viewer-qa-v013');await fs.mkdir(directory,{recursive:true});
const info=await fetch(cdp+'/json/new?about:blank',{method:'PUT'}).then(r=>r.json());
const ws=new WebSocket(info.webSocketDebuggerUrl);await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
let id=0;const pending=new Map(),errors=[];
ws.onmessage=event=>{const d=JSON.parse(event.data);if(d.id){const p=pending.get(d.id);pending.delete(d.id);if(d.error)p.reject(d.error);else p.resolve(d.result);}if(d.method==='Runtime.exceptionThrown')errors.push(d.params.exceptionDetails.text);};
const send=(method,params={})=>new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject});ws.send(JSON.stringify({id:n,method,params}));});
const evaluate=async expression=>{const r=await send('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
const delay=ms=>new Promise(r=>setTimeout(r,ms));
const screenshot=async name=>{const r=await send('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});await fs.writeFile(path.join(directory,name),Buffer.from(r.data,'base64'));};
const importText=async (text,name)=>{
  await evaluate(`(()=>{const f=new File([${JSON.stringify(text)}],${JSON.stringify(name)},{type:'text/csv'}),dt=new DataTransfer();dt.items.add(f);const input=document.getElementById('file');input.files=dt.files;input.dispatchEvent(new Event('change'));})()`);
  await delay(300);
};
try{
  await send('Page.enable');await send('Runtime.enable');
  await send('Emulation.setDeviceMetricsOverride',{width:1440,height:900,deviceScaleFactor:1,mobile:false});
  if(process.env.TTIMU_QA_CACHE_SEED==='1'){
    await send('Network.enable');
    await send('Page.navigate',{url:new URL('/__test__/cache-seed',url).href});
    for(let i=0;i<50;i++){if(await evaluate('window.__ttimuOldCachedModule===true'))break;await delay(100);}
    assert.equal(await evaluate('window.__ttimuOldCachedModule===true'),true);
    assert.equal(await evaluate('getComputedStyle(document.body).backgroundColor'),'rgb(14, 21, 31)');
    // Reload the seed document once more; its unversioned assets remain cached.
    await send('Page.navigate',{url:new URL('/__test__/cache-seed?again=1',url).href});
    for(let i=0;i<50;i++){if(await evaluate('window.__ttimuOldCachedModule===true'))break;await delay(100);}
    assert.equal(await evaluate('window.__ttimuOldCachedModule===true'),true);
    const seededAssets=await evaluate('performance.getEntriesByType("resource").filter(r=>r.name.endsWith("/style.css")||r.name.endsWith("/app.mjs")).map(r=>({name:r.name,transfer:r.transferSize,decoded:r.decodedBodySize}))');
    assert.equal(seededAssets.length,2);assert(seededAssets.every(r=>r.transfer===0&&r.decoded>0));
    console.log('Seeded old CSS/JS cache on the same origin; cache was not cleared.');
  }
  await send('Page.navigate',{url});
  for(let i=0;i<60;i++){if(await evaluate('!!document.getElementById("roll")'))break;await delay(100);}
  assert.equal(await evaluate('document.querySelectorAll("canvas").length'),7);
  assert.equal(await evaluate('document.querySelectorAll("[role=tab],.sidebar").length'),0);
  assert(await evaluate('document.querySelector(".health-strip").getBoundingClientRect().bottom<document.getElementById("plot").getBoundingClientRect().top'));
  assert(await evaluate('document.getElementById("plot").getBoundingClientRect().height>2*document.getElementById("plot-gx").getBoundingClientRect().height'));
  assert.equal(await evaluate('getComputedStyle(document.getElementById("plot")).backgroundColor'),'rgb(255, 255, 255)');
  assert.equal(await evaluate('getComputedStyle(document.body).backgroundColor'),'rgb(5, 5, 5)');
  assert.equal(await evaluate('getComputedStyle(document.getElementById("connect")).backgroundColor'),'rgb(57, 197, 187)');
  assert(await evaluate('parseFloat(getComputedStyle(document.getElementById("roll")).fontSize)>=34'));
  assert(await evaluate('(()=>{const a=document.getElementById("legend").getBoundingClientRect(),p=document.getElementById("plot").getBoundingClientRect();return Math.abs(a.x+a.width/2-p.x-p.width/2)<3;})()'));
  assert(await evaluate('document.getElementById("plot-az").getBoundingClientRect().bottom<window.innerHeight'));
  await evaluate('document.getElementById("demo").click()');await delay(2900);
  assert.match(await evaluate('document.getElementById("sourceBadge").textContent'),/MÔ PHỎNG/);
  for(const id of ['roll','pitch','yaw','gx','gy','gz','ax','ay','az','temperature'])assert.notEqual(await evaluate(`document.getElementById('${id}').textContent`),'—');
  assert.equal(await evaluate('document.getElementById("calibrate").disabled'),true);
  await evaluate('document.getElementById("window").value="all";document.getElementById("window").dispatchEvent(new Event("change"))');await delay(100);
  await screenshot('desktop.png');
  await evaluate('document.getElementById("pause").click()');await delay(100);
  const before=await evaluate('[document.getElementById("roll").textContent,document.getElementById("plot").toDataURL(),document.getElementById("sampleCount").textContent]');
  await delay(350);const after=await evaluate('[document.getElementById("roll").textContent,document.getElementById("plot").toDataURL(),document.getElementById("sampleCount").textContent]');
  assert.equal(after[0],before[0]);assert.equal(after[1],before[1]);assert.notEqual(after[2],before[2]);
  await evaluate('document.getElementById("pause").click()');
  await send('Browser.setDownloadBehavior',{behavior:'allow',downloadPath:directory});
  // A new 26-column firmware stream already has reversed roll. Do not reverse twice.
  await evaluate(`window.commands=[];window.mockPort={readable:new ReadableStream({start(c){window.serialController=c;}}),writable:new WritableStream({write(b){window.commands.push(new TextDecoder().decode(b));}}),async open(o){window.serialOptions=o;},async close(){}};Object.defineProperty(navigator,'serial',{configurable:true,value:{async requestPort(){return window.mockPort;}}});document.getElementById('connect').click();`);
  await delay(150);assert.equal(await evaluate('window.serialOptions.baudRate'),921600);
  const line=(t,ready,calib)=>`${t},-1.25,-2.5,359.9,0.01,-0.02,0.03,0.1,0.2,9.8,26.5,${ready},1000,0,${calib},0,0,1,0,0,0,0,0,0,1,-1\n`;
  await evaluate(`window.serialController.enqueue(new TextEncoder().encode(${JSON.stringify(line(1000,0,.5))}));`);await delay(150);
  assert.match(await evaluate('document.getElementById("noticeText").textContent'),/Đang hiệu chuẩn/);
  await evaluate('document.getElementById("calibrate").click()');await delay(80);assert(await evaluate('window.commands.includes("CALIBRATE\\n")'));
  await evaluate(`window.serialController.enqueue(new TextEncoder().encode(${JSON.stringify(line(2000,1,1))}));`);await delay(150);
  assert.equal(await evaluate('document.getElementById("roll").textContent'),'-1.25');
  assert.equal(await evaluate('document.getElementById("temperature").textContent'),'26.5');
  const priorFiles=new Set(await fs.readdir(directory));
  await evaluate('document.getElementById("record").click()');
  await evaluate(`window.serialController.enqueue(new TextEncoder().encode(${JSON.stringify(line(3000,1,1))}));`);await delay(100);
  await evaluate('document.getElementById("record").click()');await delay(400);
  const csv=(await fs.readdir(directory)).find(n=>n.endsWith('.csv')&&!priorFiles.has(n));assert(csv);
  const exported=await fs.readFile(path.join(directory,csv),'utf8');assert.match(exported,/3000,-1.25,-2.5,359.9/);assert.match(exported,/acc_weight,roll_sign/);
  await evaluate('document.getElementById("zero").click()');await delay(80);assert(await evaluate('window.commands.includes("ZERO_YAW\\n")'));
  await evaluate('document.getElementById("connect").click()');await delay(150);
  assert.equal(await evaluate('document.getElementById("calibrate").disabled'),true);
  // Old CSV, all channels. Hover numbers must match the exact point on all seven plots.
  await importText('t_us,roll,pitch,yaw,gx,gy,gz,ax,ay,az,temp,ready\n1000000,10,1,359,0.1,0.2,0.3,1,2,9.8,26,1\n2000000,20,2,0,0.2,0.3,0.4,2,3,9.9,27,1\n3000000,30,3,1,0.3,0.4,0.5,3,4,10,28,1\n','hover-test.csv');
  const box=await evaluate('(()=>{const r=document.getElementById("plot").getBoundingClientRect();return {x:r.x+57+(r.width-70)/2,y:r.y+70};})()');
  await send('Input.dispatchMouseEvent',{type:'mouseMoved',x:box.x,y:box.y});await delay(120);
  assert.equal(await evaluate('document.getElementById("roll").textContent'),'20.00');
  assert.equal(await evaluate('document.getElementById("yaw").textContent'),'360.00');
  assert.equal(await evaluate('document.getElementById("gx").textContent'),'0.20000');
  assert.equal(await evaluate('document.getElementById("ax").textContent'),'2.000');
  assert.match(await evaluate('document.getElementById("cursorLabel").textContent'),/2.000/);
  await send('Input.dispatchMouseEvent',{type:'mouseMoved',x:5,y:5});
  await evaluate('document.getElementById("toast").classList.remove("visible")');
  await send('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await delay(200);
  assert(await evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'));await screenshot('mobile.png');
  await send('Emulation.setDeviceMetricsOverride',{width:1440,height:900,deviceScaleFactor:1,mobile:false});
  if(process.argv[2]){await importText(await fs.readFile(process.argv[2],'utf8'),path.basename(process.argv[2]));await screenshot('provided-log.png');}
  assert.deepEqual(errors,[]);
  console.log('PASS: seven simultaneous charts, top health, compact desktop/mobile, freeze with live input, hover/value alignment, 26-column mocked USB, commands, raw CSV export/import.');
  console.log('Screenshots:',directory);
}finally{await send('Page.close');ws.close();}
