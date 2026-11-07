"""Analyze post-filtered TT telemetry; does not claim raw-data replay or new HW results.

Requires NumPy/Matplotlib. Raw input CSVs are read in place, never copied/published.
"""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
cache = ROOT / '.cache/plotting'
if cache.exists():
    sys.path.insert(0, str(cache))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(line for line in f if not line.startswith('#')))
    result = {name: np.array([float(r[name]) for r in rows]) for name in rows[0]}
    if not all(np.isfinite(v).all() for v in result.values()):
        raise ValueError('Non-finite telemetry in ' + str(path))
    if not np.all(np.diff(result['t_us']) > 0):
        raise ValueError('Split at reset/nonmonotonic timestamp before analysis')
    return result


def slope(t, values):
    tc = t - np.mean(t)
    return float(np.dot(tc, values - np.mean(values)) / np.dot(tc, tc))


def integrate(t, rates):
    return np.r_[0, np.cumsum(np.diff(t) * (rates[1:] + rates[:-1]) * 0.5)]


def analyze(path):
    d = load(path)
    t = (d['t_us'] - d['t_us'][0]) * 1e-6
    angles = {k: np.degrees(np.unwrap(np.radians(d[k]))) for k in ('roll', 'pitch', 'yaw')}
    acc = np.column_stack([d[k] for k in ('ax', 'ay', 'az')])
    norm = np.linalg.norm(acc, axis=1)
    gyro = np.column_stack([d[k] for k in ('gx', 'gy', 'gz')])
    phi, theta = np.radians(d['roll']), np.radians(d['pitch'])
    # Native INAV output yaw = minus conventional Euler yaw.
    yaw_rate = -(np.sin(phi) * gyro[:, 1] + np.cos(phi) * gyro[:, 2]) / np.cos(theta)
    gyro_heading = np.degrees(integrate(t, yaw_rate))
    estimated_gravity = np.column_stack([-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)])
    weight = np.exp(-((norm / 9.80665 - 1) ** 2) / (2 * 0.2 ** 2))
    weight *= np.where(d['t_us'] < 20e6, 10, 1)  # Default unarmed FC profile.
    proportional = np.cross(acc / norm[:, None], estimated_gravity) * (0.2 * weight[:, None])
    p_rate = -(np.sin(phi) * proportional[:, 1] + np.cos(phi) * proportional[:, 2]) / np.cos(theta)
    gyro_p_heading = np.degrees(integrate(t, yaw_rate + p_rate))
    residual = angles['yaw'] - angles['yaw'][0] - gyro_p_heading
    summary = {
        'file': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'samples': len(t), 'duration_s': float(t[-1]),
        'device_time_s': [float(d['t_us'][0]/1e6), float(d['t_us'][-1]/1e6)],
        'csv_rate_hz': (len(t)-1)/float(t[-1]),
        'angles': {k: {'first_deg': float(v[0]), 'last_unwrapped_deg': float(v[-1]),
                        'delta_deg': float(v[-1]-v[0]), 'linear_rate_deg_per_min': slope(t,v)*60} for k,v in angles.items()},
        'acc_mean_mps2': np.mean(acc,axis=0).tolist(),
        'acc_norm_mean_mps2': float(np.mean(norm)), 'acc_norm_sd_mps2': float(np.std(norm)),
        'gyro_mean_deg_per_s': np.degrees(np.mean(gyro,axis=0)).tolist(),
        'gyro_sd_deg_per_s': np.degrees(np.std(gyro,axis=0)).tolist(),
        'temperature_c': [float(np.min(d['temp'])),float(np.max(d['temp']))],
        'logged_dt_us': [float(np.min(d['dt_us'])),float(np.mean(d['dt_us'])),float(np.max(d['dt_us']))],
        'gap_counter': [float(d['gaps'][0]),float(d['gaps'][-1])],
        'late_counter': [float(d['late'][0]),float(d['late'][-1])],
        'calibration_retry_counter': [float(d['retries'][0]),float(d['retries'][-1])],
        'ready_fraction': float(np.mean(d['ready'])),
        'gyro_only_heading_rate_deg_per_min': slope(t,gyro_heading)*60,
        'approx_gyro_plus_p_heading_rate_deg_per_min': slope(t,gyro_p_heading)*60,
        'unlogged_correction_heading_rate_deg_per_min': slope(t,residual)*60,
        'limits': '100 Hz post-filtered data; no raw counts, bias values, quaternion, I feedback or boot interval. Kinematic/P estimates are approximate; unarmed default profile assumed.'
    }
    return d,t,angles,gyro_heading,gyro_p_heading,summary


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('logs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'reports/2026-11-07-stationary')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    results=[analyze(p) for p in args.logs]
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':0.2})
    fig,axes=plt.subplots(4,len(results),figsize=(14,11),squeeze=False,layout='constrained')
    for col,(d,t,a,gh,gph,s) in enumerate(results):
        axes[0,col].set_title(f"Log {col+1}: {s['duration_s']:.2f} s stationary (recorded)")
        for k in ('roll','pitch','yaw'):
            v=a[k]-a[k][0] if k=='yaw' else a[k]
            axes[0,col].plot(t,v,label='Yaw change' if k=='yaw' else k.capitalize())
        axes[0,col].set_ylabel('Angle (deg, unwrapped)');axes[0,col].legend()
        for k in ('ax','ay','az'): axes[1,col].plot(t,d[k],label=k)
        axes[1,col].set_ylabel('Acceleration (m/s²)');axes[1,col].legend()
        for k in ('gx','gy','gz'): axes[2,col].plot(t,np.degrees(d[k]),label=k,linewidth=.6,alpha=.8)
        axes[2,col].set_ylabel('Logged gyro (deg/s)');axes[2,col].legend()
        axes[3,col].plot(t,a['yaw']-a['yaw'][0],label='Recorded yaw change')
        axes[3,col].plot(t,gh,label='From logged gyro only')
        axes[3,col].plot(t,gph,'--',label='Approx. gyro + P feedback')
        axes[3,col].set_ylabel('Heading change (deg)');axes[3,col].set_xlabel('Time in recording (s)');axes[3,col].legend()
    fig.suptitle('TT IMU v0.1.1 — supplied stationary logs; no new hardware measurements',fontsize=14)
    fig.savefig(args.output/'stationary-logs.png',dpi=150);plt.close(fig)
    summaries=[r[-1] for r in results]
    (args.output/'metrics.json').write_text(json.dumps({'measured_logs':summaries,'note':'startup-model.csv is a separate synthetic zero-gyro experiment, not raw log replay.'},indent=2)+'\n',encoding='utf-8')
    model=args.output/'startup-model.csv'
    if model.exists():
        m=np.genfromtxt(model,delimiter=',',names=True)
        fig,axes=plt.subplots(2,1,figsize=(10,7),layout='constrained')
        for key in ('old_roll','old_pitch','old_yaw','new_roll','new_pitch','new_yaw'):
            axes[0].plot(m['t'],np.degrees(np.unwrap(np.radians(m[key]))),label=key,linestyle='--' if key.startswith('new') else '-')
        axes[0].set_ylabel('Angle (deg)');axes[0].legend(ncol=3)
        axes[1].plot(m['t'],np.degrees(m['old_iz']),label='Old integral Z')
        axes[1].plot(m['t'],np.degrees(m['new_iz']),label='Seeded + corrected axes integral Z')
        axes[1].set_xlabel('Model time (s)');axes[1].set_ylabel('Correction (deg/s)');axes[1].legend()
        fig.suptitle('Synthetic regression: zero gyro, constant log-1 mean acceleration\nOld initial pose from log; integral starts at zero. Not raw replay or predicted physical accuracy.')
        fig.savefig(args.output/'startup-regression.png',dpi=150);plt.close(fig)
    for s in summaries:
        print(s['file'],json.dumps({k:s[k] for k in ['duration_s','angles','gyro_only_heading_rate_deg_per_min','approx_gyro_plus_p_heading_rate_deg_per_min','unlogged_correction_heading_rate_deg_per_min']},ensure_ascii=False))

if __name__=='__main__':main()
