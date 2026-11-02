import test from 'node:test';
import assert from 'node:assert/strict';
import {TelemetryParser, TraceBuffer, COLUMNS, csvRow} from '../viewer/telemetry.mjs';

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
  p.push(row); p.finish(); assert.equal(samples.length, 1); assert.equal(samples[0].az, 9.8); assert.equal(csvRow(samples[0]), row);
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
  assert.equal(COLUMNS.length,17);
});
