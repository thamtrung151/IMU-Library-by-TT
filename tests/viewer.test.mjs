import test from 'node:test';
import assert from 'node:assert/strict';
import {TelemetryParser, TraceBuffer, COLUMNS, csvRow, displayValue} from '../viewer/telemetry.mjs';

test('USB chunks, CRLF, comments and split header are decoded without losing a sample', () => {
  const samples = [], info = [];
  const p = new TelemetryParser(s => samples.push(s), s => info.push(s));
  for (const chunk of ['# ICM42688\r\nt_', 'us,roll,pitch,yaw,ready\r', '\n1000,-1.2,2.3,359.9,0\n2', '000,0,1,0.1,1\r\n']) p.push(chunk);
  assert.equal(samples.length, 2); assert.equal(samples[0].roll, -1.2); assert.equal(samples[1].ready, 1); assert.equal(info[0], 'ICM42688');
});
test('invalid finite fields, incomplete rows and invalid readiness do not become fake data', () => {
  const samples = []; const p = new TelemetryParser(s => samples.push(s));
  p.push('t_us,roll,pitch,yaw,ready\n1000,0,0,NaN,1\n2000,,0,2,1\n3000,0,0,1,9\n4000,0,0,1\n');
  assert.equal(samples.length, 0); assert.equal(p.errors, 4);
});
test('reconnect without header accepts the full example protocol, and final CSV line needs no newline', () => {
  const samples = []; const p = new TelemetryParser(s => samples.push(s));
  const row = [1234,1,2,3,0,0,0,0,0,9.8,25,1,1000,0,1,0,0].join(',');
  p.push(row); p.finish(); assert.equal(samples.length, 1); assert.equal(samples[0].az, 9.8);
  assert.equal(csvRow(samples[0]).split(',').slice(0,17).join(','), row);
  assert.ok(csvRow(samples[0]).split(',').slice(17).every(v=>v===''));
});
test('yaw unwraps through 360 and the ring is bounded in chronological order', () => {
  const b = new TraceBuffer(3);
  for (const [i,yaw] of [359,0,1,2].entries()) b.add({t_us:i*1000,yaw});
  assert.deepEqual(b.values().map(v => v.yaw_unwrapped), [360,361,362]); assert.equal(b.length,3);
  b.add({t_us:0,yaw:10}); assert.equal(b.length,1); assert.equal(b.values()[0].yaw_unwrapped,10);
});
test('oversized / corrupt USB line is discarded and parsing resumes on the following line', () => {
  const samples = []; const p = new TelemetryParser(s => samples.push(s));
  p.push('x'.repeat(5000)); p.push('tail\nt_us,roll,pitch,yaw\n0,0,0,0\n');
  assert.equal(samples.length,1); assert.equal(p.errors,1);
});
test('CSV header order is honored and a repeated header can change optional fields', () => {
  const samples=[];const p=new TelemetryParser(s=>samples.push(s));
  p.push('yaw,t_us,roll,pitch\n'); // Header must begin with t_us, as documented.
  p.push('t_us,yaw,pitch,roll\n1000,90,2,-5\nt_us,roll,pitch,yaw,temp\n2000,1,2,3,\n');
  assert.equal(samples.length,2);assert.equal(samples[0].roll,-5);assert.equal(samples[1].temp,null);
});

test('new firmware diagnostic columns survive a headerless USB reconnect and CSV round trip', () => {
  const samples=[];const p=new TelemetryParser(s=>samples.push(s));
  const values=[1234,1,2,3,0,0,0,0,0,9.8,25,1,1000,0,1,0,0,1,0,0,0,0.001,-0.002,0.003,0.99,-1,7];
  const row=values.join(',');p.push(row+'\n');
  assert.equal(samples.length,1);assert.equal(samples[0].iz,0.003);assert.equal(samples[0].qw,1);
  assert.equal(samples[0].acc_weight,0.99);assert.equal(samples[0].roll_sign,-1);assert.equal(csvRow(samples[0]),row);
  p.push(COLUMNS.join(',')+'\n'+row+'\n');assert.equal(samples.length,2);
  assert.equal(samples[0].reference_id,7);
  p.reset();p.push(values.slice(0,26).join(',')+'\n');assert.equal(samples.length,3);assert.equal(samples[2].reference_id,undefined);
});

test('zero-reference changes and calibration completion restart continuous angles without changing CSV data', () => {
  const b=new TraceBuffer(10);
  b.add({t_us:0,roll:179,pitch:0,yaw:359,ready:1,reference_id:0});
  assert.equal(b.add({t_us:1000,roll:-179,pitch:0,yaw:1,ready:1,reference_id:0}).yaw_unwrapped,361);
  const zero=b.add({t_us:2000,roll:0,pitch:0,yaw:0,ready:1,reference_id:1});
  assert.equal(zero.yaw_unwrapped,0);assert.equal(zero.roll_unwrapped,0);assert.equal(zero.reference_changed,true);
  b.add({t_us:3000,roll:0,pitch:0,yaw:359,ready:0,reference_id:2});
  const done=b.add({t_us:4000,roll:0,pitch:0,yaw:0,ready:1,reference_id:3});
  assert.equal(done.yaw_unwrapped,0);assert.equal(done.reference_changed,true);
  assert.equal(csvRow(done).split(',').at(-1),'3');assert.equal(csvRow(done).split(',')[3],'0');
  const p=new TelemetryParser(()=>assert.fail('invalid reference accepted'));
  for(const id of [-1,.5,4294967296])p.push('t_us,roll,pitch,yaw,reference_id\n0,0,0,0,'+id+'\n');
  assert.equal(p.errors,3);
});

test('old diagnostic firmware still reconnects without a header; roll unwrap matches the plotted number', () => {
  const samples=[];const p=new TelemetryParser(s=>samples.push(s));
  p.push([1234,179,0,359,0,0,0,0,0,9.8,25,1,1000,0,1,0,0,1,0,0,0,0,0,0,1].join(',')+'\n');
  assert.equal(samples.length,1);assert.equal(samples[0].roll_sign,undefined);
  const b=new TraceBuffer(3);b.add(samples[0]);b.add({...samples[0],t_us:2000,roll:-179,yaw:1});
  assert.equal(b.values()[1].roll_unwrapped,181);assert.equal(b.values()[1].yaw_unwrapped,361);
  const bad=[1234,0,0,0,0,0,0,0,0,9.8,25,1,1000,0,1,0,0,1,0,0,0,0,0,0,1,0].join(',');
  p.push(bad+'\n');assert.equal(p.errors,1);
});

test('standard angles stay bounded while continuous mode explicitly keeps accumulated turns', () => {
  const sample={t_us:0,roll:-4.34,roll_unwrapped:355.66,pitch:.69,yaw:9.25,yaw_unwrapped:369.25};
  assert.equal(displayValue(sample,'yaw'),9.25);assert.equal(displayValue(sample,'yaw',true),369.25);
  assert.ok(Math.abs(displayValue(sample,'roll')+4.34)<1e-10);
  assert.equal(displayValue(sample,'roll',true),355.66);
  assert.equal(displayValue({...sample,yaw:360},'yaw'),0);
  assert.equal(displayValue({...sample,yaw:-1},'yaw'),359);
  assert.equal(sample.yaw,9.25);assert.equal(csvRow(sample).split(',')[3],'9.25');
  const b=new TraceBuffer(3);b.add({t_us:0,roll:179,pitch:0,yaw:359});b.add({t_us:1000,roll:-179,pitch:0,yaw:1});
  const p=b.values()[1];assert.equal(displayValue(p,'yaw'),1);assert.equal(displayValue(p,'yaw',true),361);
  assert.equal(displayValue(p,'roll'),-179);assert.equal(displayValue(p,'roll',true),181);
});
