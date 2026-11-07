"""Build a test-only reference from the full pinned upstream Mahony function.

No network, audit checkout or firmware build is required for end users.
The 6-axis port is compared to the FULL function with MAG/COG absent.
"""
import argparse
import hashlib
import json
from pathlib import Path
from vendor_inav import function, adapt_helpers

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    vendor=ROOT/'vendor/inav-9.0.1'
    manifest=json.loads((vendor/'manifest.json').read_text())
    for item in manifest['files']:
        if hashlib.sha256((vendor/item['file']).read_bytes()).hexdigest()!=item['sha256']:
            raise SystemExit('Changed upstream snapshot: '+item['file'])
    source=(vendor/'flight/imu.c').read_text(encoding='utf-8')
    mahony=function(source,'imuMahonyAHRSupdate')
    mahony=mahony.replace('STATIC_FASTRAM fpVector3_t vGyroDriftEstimate = { 0 };','fpVector3_t &vGyroDriftEstimate = drift;')
    matrix=function(source,'imuComputeRotationMatrix').replace('STATIC_UNIT_TESTED','')
    text='''// GPL-3.0-or-later. Test reference extracted from pinned INAV 9.0.1.
#include "reference.h"
#include "ttimu/detail/InavVectors.h"
#define STATIC_FASTRAM static
#define STATIC_UNIT_TESTED
#define DEGREES_TO_RADIANS(x) ((x)*(M_PIf/180.0f))
#define SPIN_RATE_LIMIT 20
#define MAX_MAG_NEARNESS 0.25f
#define COS10DEG 0.985f
#define COS20DEG 0.940f
#define STATE(x) false
#define MULTIROTOR 0
namespace reference {
using namespace ttimu::detail;
static fpQuaternion_t orientation;
static fpVector3_t drift;
static float rMat[3][3],accWeight;
static pt1Filter_t rates[3];
static ImuRuntimeConfigStubPlaceholder;
struct Runtime {float dcm_kp_acc=0.2f,dcm_ki_acc=0.005f,dcm_kp_mag=0.2f,dcm_ki_mag=0.005f;} imuRuntimeConfig;
struct Config {float gps_yaw_weight=100;} config;
static Config* imuConfig(){return &config;}
static bool isMixerTransitionMixing=false;
struct GPS {float groundSpeed=0;} gpsSol;
static fpVector3_t vCorrectedMagNorth={{1,0,0}};
static float imuCalculateMcCogWeight(){return 0;}
static float imuCalculateMcCogAccWeight(){return 0;}
// Valid-data equivalence only; invalid-input recovery is tested on the public API.
static void imuCheckAndResetOrientationQuaternion(const fpQuaternion_t*,const fpVector3_t*){}
'''.replace('static ImuRuntimeConfigStubPlaceholder;\n','')+matrix+'\n'+mahony+'''
void reset(){orientation={1,0,0,0};drift={{0,0,0}};accWeight=0;std::memset(rates,0,sizeof(rates));imuComputeRotationMatrix();}
void step(const fpVector3_t& gyro,const fpVector3_t& acc,float dt,uint32_t time,bool armed){
    float rotationSq=0;
    for(unsigned a=0;a<3;++a){float r=pt1FilterApply4(&rates[a],gyro.v[a],3,dt);rotationSq+=r*r;}
    fpVector3_t normalized;
    vectorScale(&normalized,&acc,1.0f/Gravity);
    accWeight=(!armed&&time<20000?10.0f:1.0f)*bellCurve(fast_fsqrtf(vectorNormSquared(&normalized))-1,0.2f);
    if(armed){float r=fast_fsqrtf(rotationSq)/1.001f;accWeight*=scaleRangef(constrainf(r,DEGREES_TO_RADIANS(10),DEGREES_TO_RADIANS(20)),DEGREES_TO_RADIANS(10),DEGREES_TO_RADIANS(20),1,0);}
    imuMahonyAHRSupdate(dt,&gyro,accWeight>0.001f?&acc:nullptr,nullptr,nullptr,nullptr,accWeight,1);
}
fpQuaternion_t quaternion(){return orientation;}
fpVector3_t integral(){return drift;}
float weight(){return accWeight;}
}
'''
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(adapt_helpers(text),encoding='utf-8')
    print('Generated independent full-function INAV reference from verified vendor snapshots.')
if __name__=='__main__':main()
