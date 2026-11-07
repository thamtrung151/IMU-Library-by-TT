// SPDX-License-Identifier: GPL-3.0-or-later
const LEGACY_COLUMNS = ['t_us','roll','pitch','yaw','gx','gy','gz','ax','ay','az','temp','ready','dt_us','gaps','calib','retries','late'];
const DIAGNOSTIC_COLUMNS = [...LEGACY_COLUMNS,'qw','qx','qy','qz','ix','iy','iz','acc_weight'];
export const COLUMNS = [...DIAGNOSTIC_COLUMNS,'roll_sign'];
const REQUIRED = ['t_us','roll','pitch','yaw'];

export class TelemetryParser {
  constructor(onSample, onInfo = () => {}) {
    this.onSample = onSample; this.onInfo = onInfo; this.reset();
  }
  reset() { this.pending = ''; this.header = null; this.errors = 0; this.discarding = false; }
  push(text) {
    this.pending += text;
    let end;
    while ((end = this.pending.indexOf('\n')) !== -1) {
      const line = this.pending.slice(0, end).replace(/\r$/, '');
      this.pending = this.pending.slice(end + 1);
      if (this.discarding) { this.discarding = false; continue; }
      if (line.length > 4096) { this.errors++; continue; }
      this.line(line.trim());
    }
    if (this.pending.length > 4096) { this.pending = ''; this.discarding = true; this.errors++; }
  }
  finish() { if (this.pending && !this.discarding) this.line(this.pending.trim()); this.pending = ''; this.discarding = false; }
  line(line) {
    if (!line) return;
    if (line.startsWith('#')) { this.onInfo(line.slice(1).trim()); return; }
    const fields = line.split(',').map(s => s.trim());
    if (fields[0] === 't_us') {
      this.header = REQUIRED.every(k => fields.includes(k)) && new Set(fields).size === fields.length ? fields : null;
      if (!this.header) this.errors++;
      return;
    }
    // The example emits a header once; reconnects can begin in the middle of a stream.
    const header = this.header ?? (fields.length === COLUMNS.length ? COLUMNS : fields.length === DIAGNOSTIC_COLUMNS.length ? DIAGNOSTIC_COLUMNS : fields.length === LEGACY_COLUMNS.length ? LEGACY_COLUMNS : null);
    if (!header || fields.length !== header.length) { this.errors++; return; }
    const sample = {};
    for (let i = 0; i < header.length; i++) {
      if (!fields[i]) { sample[header[i]] = null; continue; }
      const value = Number(fields[i]);
      if (!Number.isFinite(value)) { this.errors++; return; }
      sample[header[i]] = value;
    }
    if (!REQUIRED.every(k => Number.isFinite(sample[k])) || !Number.isSafeInteger(sample.t_us) || sample.t_us < 0) { this.errors++; return; }
    if (sample.ready !== undefined && sample.ready !== null && sample.ready !== 0 && sample.ready !== 1) { this.errors++; return; }
    if (sample.roll_sign !== undefined && sample.roll_sign !== null && sample.roll_sign !== 1 && sample.roll_sign !== -1) { this.errors++; return; }
    this.onSample(sample);
  }
}

export class TraceBuffer {
  constructor(capacity = 10000) { this.capacity = capacity; this.clear(); }
  clear() { this.points = new Array(this.capacity); this.start = 0; this.length = 0; this.lastYaw = null; this.yawUnwrapped = 0; this.lastRoll = null; this.rollUnwrapped = 0; this.lastTime = null; }
  add(sample) {
    if (this.lastTime !== null && sample.t_us < this.lastTime) this.clear(); // Device reset or a new CSV run.
    let yaw = sample.yaw;
    if (this.lastYaw !== null) {
      let change = sample.yaw - this.lastYaw;
      change = ((change + 180) % 360 + 360) % 360 - 180;
      this.yawUnwrapped += change;
    } else this.yawUnwrapped = yaw;
    this.lastYaw = yaw; this.lastTime = sample.t_us;
    let roll=sample.roll;
    if(Number.isFinite(roll)){
      this.rollUnwrapped=this.lastRoll===null?roll:this.rollUnwrapped+((roll-this.lastRoll+180)%360+360)%360-180;
      this.lastRoll=roll;
    }
    const point = {...sample, yaw_unwrapped: this.yawUnwrapped, roll_unwrapped:Number.isFinite(roll)?this.rollUnwrapped:undefined};
    const index = (this.start + this.length) % this.capacity;
    this.points[index] = point;
    if (this.length < this.capacity) this.length++; else this.start = (this.start + 1) % this.capacity;
    return point;
  }
  values() { return Array.from({length: this.length}, (_, i) => this.points[(this.start + i) % this.capacity]); }
}

export function csvRow(s) { return COLUMNS.map(k => Number.isFinite(s[k]) ? s[k] : '').join(','); }
