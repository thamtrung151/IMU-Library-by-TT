"""Reproduce the small, attributed INAV 9.0.1 port from pinned source.

Default source: local audit checkout; --source allows an external INAV checkout.
The published vendor snapshots make the release and tests self-contained.
"""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHA = 'd44f2cf6e60cc9318b10551291b596370b5c957b'

def adapt_helpers(text):
    # Keep Arduino.h macros intact, regardless of user include order.
    names = {'sq': 'inavSquare', 'radians': 'inavRadians'}
    return re.sub(r'\b(sq|radians)(?=\s*\()', lambda m: names[m[1]], text)

def function(text, name):
    masked = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"', lambda m: re.sub(r'[^\n]', ' ', m[0]), text, flags=re.S)
    m = re.search(r'\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{', masked)
    if not m:
        raise ValueError(name)
    start = text.rfind('\n', 0, m.start()) + 1
    pos = masked.index('{', m.start())
    depth = 1
    end = pos + 1
    while depth:
        depth += (masked[end] == '{') - (masked[end] == '}')
        end += 1
    return text[start:end]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=ROOT/'research/upstream/inav9')
    args = parser.parse_args()
    source = args.source
    if subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip() != SHA:
        raise SystemExit('INAV checkout does not match the pinned SHA')
    paths = ['flight/imu.c','common/maths.c','common/maths.h','common/vector.h',
             'common/quaternion.h','common/filter.c','common/filter.h',
             'common/calibration.c','flight/kalman.c','flight/kalman.h',
             'drivers/accgyro/accgyro_icm42605.c','drivers/accgyro/accgyro_mpu6500.c',
             'fc/settings.yaml']
    sources = {}
    inventory = []
    for rel in paths:
        raw = subprocess.check_output(['git','-C',str(source),'show',f'HEAD:src/main/{rel}'])
        target = ROOT/'vendor/inav-9.0.1'/rel
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(raw)
        sources[rel] = raw.decode()
        inventory.append({'file': rel, 'sha256':hashlib.sha256(raw).hexdigest()})
    (ROOT/'LICENSE').write_bytes(subprocess.check_output(['git','-C',str(source),'show','HEAD:LICENSE']))
    detail = ROOT/'src/ttimu/detail'
    notice = '// SPDX-License-Identifier: GPL-3.0-or-later\n// Adapted from INAV 9.0.1, '+SHA+'; see THIRD_PARTY_NOTICES.md.\n'
    vector_names = ['vectorZero','vectorNormSquared','vectorNormalize','vectorCrossProduct','vectorAdd','vectorScale']
    quat_names = ['quaternionInitUnit','quaternionInitFromVector','quaternionNormSqared','quaternionMultiply','quaternionScale','quaternionConjugate','quaternionNormalize','quaternionRotateVector','quaternionRotateVectorInv']
    vectors = [function(sources['common/vector.h'], n) for n in vector_names]
    quats = [function(sources['common/quaternion.h'], n) for n in quat_names]
    (detail/'InavVectors.h').write_text(notice+'#pragma once\n#include "InavMath.h"\nnamespace ttimu { namespace detail {\n'+'\n\n'.join(vectors+quats)+'\n}}\n',encoding='utf-8')
    math_names = ['sin_approx','cos_approx','atan2_approx','acos_approx','fast_fsqrtf','constrainf','scaleRangef','gaussian','bellCurve','devClear','devPush','devVariance','devStandardDeviation']
    defs = '''#define sinPolyCoef3 -1.666665710e-1f
#define sinPolyCoef5 8.333017292e-3f
#define sinPolyCoef7 -1.980661520e-4f
#define sinPolyCoef9 2.600054768e-6f
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define FAST_CODE
#define NOINLINE
#define SITL_BUILD
#define BIQUAD_Q (1.0f / sqrtf(2.0f))
'''
    math = [function(sources['common/maths.c'], n) for n in math_names]
    filters = [function(sources['common/filter.c'], n) for n in ['pt1ComputeRC','pt1FilterInitRC','pt1FilterInit','pt1FilterApply','pt1FilterApply4','pt1FilterReset','biquadFilterSetupPassthrough','biquadFilterInit','biquadFilterApply','biquadFilterReset']]
    kalman = [function(sources['flight/kalman.c'],n).replace('static void ','void ') for n in ['gyroKalmanInitAxis','kalman_process','updateAxisVariance']]
    # Match CMSIS arm_sqrt_f32's negative-input result on non-ARM hosts/Xtensa.
    kalman[-1] = re.sub(r'#if !defined\(SITL_BUILD\).*?#endif', 'float squirt = kalmanState->axisVar < 0.0f ? 0.0f : sqrtf(kalmanState->axisVar);', kalman[-1],flags=re.S)
    (detail/'InavMath.cpp').write_text(notice+'#include "InavMath.h"\n'+defs+'namespace ttimu { namespace detail {\nusing std::isnan;\n'+'\n\n'.join(math+filters+kalman)+'\n}}\n',encoding='utf-8')
    imu = sources['flight/imu.c']
    matrix = function(imu,'imuComputeRotationMatrix').replace('STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)','void InavCore::computeRotationMatrix()')
    start = imu.index('    /* Step 2: Roll and pitch correction')
    end = imu.index('    // Check for invalid quaternion',start)
    integration = imu[start:end]
    core = notice+'''#include "InavCore.h"
#define DEGREES_TO_RADIANS(x) ((x) * (M_PIf / 180.0f))
#define SPIN_RATE_LIMIT 20
namespace ttimu { namespace detail {
void InavCore::reset() {
    orientation = {1,0,0,0}; vGyroDriftEstimate = {{0,0,0}};
    std::memset(rotationFilters_, 0, sizeof(rotationFilters_));
    lastAccWeight = 0; recoveries = 0; computeRotationMatrix();
}
'''+matrix+'''
void InavCore::integrate(float dt, const fpVector3_t& gyroRad,
                         const fpVector3_t* accBF, float accWScaler) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(accWScaler) ||
        !std::isfinite(gyroRad.x) || !std::isfinite(gyroRad.y) || !std::isfinite(gyroRad.z) ||
        (accBF && (!std::isfinite(accBF->x) || !std::isfinite(accBF->y) || !std::isfinite(accBF->z)))) {
        ++recoveries; return;
    }
    const fpQuaternion_t previous = orientation;
    const fpVector3_t previousDrift = vGyroDriftEstimate;
    fpVector3_t vRotation = gyroRad;
    const float spin_rate_sq = vectorNormSquared(&vRotation);
'''+integration+'''
    // Port safety guard: restore the previous state on invalid arithmetic.
    const float norm = quaternionNormSqared(&orientation);
    if (!std::isfinite(norm) || norm < 1e-12f) {
        orientation = previous; vGyroDriftEstimate = previousDrift; ++recoveries;
    }
    computeRotationMatrix();
}
void InavCore::step(const fpVector3_t& gyroRad, const fpVector3_t& accCmss,
                    float dt, uint32_t uptimeMs, bool armed) {
    float rotationSq = 0;
    for (unsigned axis=0; axis<3; ++axis) {
        float r = pt1FilterApply4(&rotationFilters_[axis], gyroRad.v[axis], 3, dt);
        rotationSq += r*r;
    }
    fpVector3_t normalized;
    vectorScale(&normalized, &accCmss, 1.0f / Gravity);
    float weight = bellCurve(fast_fsqrtf(vectorNormSquared(&normalized))-1.0f, 0.2f);
    weight *= (!armed && uptimeMs < 20000) ? 10.0f : 1.0f;
    if (armed) {
        const float rate = fast_fsqrtf(rotationSq) / 1.001f;
        weight *= scaleRangef(constrainf(rate,radians(10),radians(20)),radians(10),radians(20),1,0);
    }
    lastAccWeight = weight;
    integrate(dt,gyroRad,weight > 0.001f ? &accCmss : nullptr,weight);
}
void InavCore::zeroYaw() {
    const float angle = -atan2_approx(rMat[1][0],rMat[0][0]);
    fpQuaternion_t offset = {cos_approx(angle*0.5f),0,0,sin_approx(angle*0.5f)};
    quaternionMultiply(&orientation,&offset,&orientation);
    quaternionNormalize(&orientation,&orientation);
    computeRotationMatrix();
}
}}
'''
    (detail/'InavCore.cpp').write_text(core,encoding='utf-8')
    for generated in ('InavCore.cpp','InavMath.cpp','InavVectors.h'):
        target=detail/generated
        adapted=adapt_helpers(target.read_text(encoding='utf-8'))
        target.write_text('\n'.join(line.rstrip() for line in adapted.splitlines())+'\n',encoding='utf-8')
    (ROOT/'vendor/inav-9.0.1/manifest.json').write_text(json.dumps({'repository':'https://github.com/iNavFlight/inav','tag':'9.0.1','commit':SHA,'files':inventory},indent=2)+'\n',encoding='utf-8')
    print('Vendored pinned INAV source and generated math / six-axis core.')

if __name__ == '__main__':
    main()
