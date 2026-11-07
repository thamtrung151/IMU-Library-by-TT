"""Create an Arduino-installable ZIP from tracked release files, not local caches."""
import argparse
import json
import subprocess
import zipfile
from pathlib import Path

root=Path(__file__).resolve().parents[1]
version=json.loads((root/'library.json').read_text(encoding='utf-8'))['version']
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=root/f'.cache/releases/IMU-Library-by-TT-v{version}.zip');args=p.parse_args()
tracked=subprocess.check_output(['git','-C',str(root),'ls-files','-z']).decode().split('\0')
args.output.parent.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED) as archive:
    for name in tracked:
        if not name or name.startswith('.github/'):
            continue
        archive.write(root/name,'IMU-Library-by-TT/'+name)
print(args.output)
