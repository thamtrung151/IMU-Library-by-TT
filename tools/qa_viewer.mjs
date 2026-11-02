// Local QA against an isolated Chromium/Edge instance, never a user's browser profile.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
const root=path.resolve('.');
const info=await fetch('http://127.0.0.1:9226/json/new?about:blank',{method:'PUT'}).then(r=>r.json());
const ws=new WebSocket(info.webSocketDebuggerUrl);
await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
let id=0;const pending=new Map(),errors=[];
ws.onmessage=event=>{
  const data=JSON.parse(event.data);
  if(data.id){const waiter=pending.get(data.id);pending.delete(data.id);if(data.error)waiter.reject(data.error);else waiter.resolve(data.result);}
  if(data.method==='Runtime.exceptionThrown')errors.push(data.params.exceptionDetails.text);
};
const send=(method,params={})=>new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject});ws.send(JSON.stringify({id:n,method,params}));});
const evaluate=async expression=>{const r=await send('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
await send('Page.enable');await send('Runtime.enable');
await send('Emulation.setDeviceMetricsOverride',{width:1440,height:1100,deviceScaleFactor:1,mobile:false});
await send('Page.navigate',{url:'http://127.0.0.1:8080'});
for(let i=0;i<50;i++){if(await evaluate('!!document.getElementById("emptyDemo")'))break;await delay(100);}
await delay(250);
assert.equal(await evaluate('document.getElementById("empty").hidden'),false);
await evaluate('document.getElementById("demo").click()');await delay(2400);
assert.match(await evaluate('document.getElementById("sourceBadge").textContent'),/MÔ PHỎNG/);
assert.notEqual(await evaluate('document.getElementById("roll").textContent'),'—');
assert.equal(await evaluate('document.getElementById("calibrate").disabled'),true);
await evaluate('document.querySelector("[data-mode=gyro]").click()');await delay(100);
assert.match(await evaluate('document.getElementById("chartUnit").textContent'),/rad\/s/);
await evaluate('document.querySelector("[data-mode=attitude]").click()');
await delay(6200); // Allow the correct plot to repaint and the initial toast to disappear.
const directory=path.join(root,'.cache','viewer-qa');await fs.mkdir(directory,{recursive:true});
await send('Browser.setDownloadBehavior',{behavior:'allow',downloadPath:directory});
const screenshot=async name=>{const data=await send('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});await fs.writeFile(path.join(directory,name),Buffer.from(data.data,'base64'));};
await screenshot('desktop.png');
await send('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await delay(250);
assert(await evaluate('document.documentElement.scrollWidth <= window.innerWidth+1'));
await screenshot('mobile.png');
await send('Emulation.setDeviceMetricsOverride',{width:1440,height:1100,deviceScaleFactor:1,mobile:false});

// Test Web Serial UI with an explicitly mocked port, not a real IMU.
await evaluate(`window.commands=[];window.mockPort={
  readable:new ReadableStream({start(c){window.serialController=c;}}),
  writable:new WritableStream({write(bytes){window.commands.push(new TextDecoder().decode(bytes));}}),
  async open(options){window.serialOptions=options;},async close(){}
};Object.defineProperty(navigator,'serial',{configurable:true,value:{async requestPort(){return window.mockPort;}}});
document.getElementById('connect').click();`);
await delay(150);
assert.equal(await evaluate('window.serialOptions.baudRate'),921600);
const line=(time,ready,calib)=>`${time},1.25,-2.5,359.9,0,0,0,0,0,9.8,26.5,${ready},1000,0,${calib},0,0\n`;
await evaluate(`window.serialController.enqueue(new TextEncoder().encode(${JSON.stringify(line(1000,0,0.5))}));`);await delay(150);
assert.match(await evaluate('document.getElementById("noticeText").textContent'),/Đang hiệu chuẩn/);
assert.equal(await evaluate('document.getElementById("calibrate").disabled'),false);
await evaluate('document.getElementById("calibrate").click()');await delay(100);
assert(await evaluate('window.commands.includes("CALIBRATE\\n")'));
await evaluate(`window.serialController.enqueue(new TextEncoder().encode(${JSON.stringify(line(2000,1,1))}));`);await delay(150);
assert.match(await evaluate('document.getElementById("noticeText").textContent'),/Dữ liệu sẵn sàng/);
await evaluate('document.getElementById("record").click()');
await evaluate(`window.serialController.enqueue(new TextEncoder().encode(${JSON.stringify(line(3000,1,1))}));`);await delay(100);
await evaluate('document.getElementById("record").click()');await delay(500);
const csv=(await fs.readdir(directory)).find(name=>name.endsWith('.csv'));
assert(csv,'Recording must download a CSV');
assert.match(await fs.readFile(path.join(directory,csv),'utf8'),/3000,1.25,-2.5,359.9/);
await evaluate('document.getElementById("zero").click()');await delay(100);
assert(await evaluate('window.commands.includes("ZERO_YAW\\n")'));
await evaluate('document.getElementById("connect").click()');await delay(200);
assert.equal(await evaluate('document.getElementById("calibrate").disabled'),true);
await evaluate(`const f=new File(['t_us,roll,pitch,yaw,ready\\n1000,3,4,5,1\\n'], 'test.csv',{type:'text/csv'});const dt=new DataTransfer();dt.items.add(f);const input=document.getElementById('file');input.files=dt.files;input.dispatchEvent(new Event('change'));`);
await delay(200);
assert.match(await evaluate('document.getElementById("sourceBadge").textContent'),/CSV/);
assert.equal(await evaluate('document.getElementById("roll").textContent'),'3.00');
assert.deepEqual(errors,[]);
console.log('PASS: real browser desktop/mobile, demo labels, mocked serial/calibration/zero-yaw, CSV download/import, no JS exceptions.');
console.log('Screenshots:',directory);
await send('Page.close');ws.close();
