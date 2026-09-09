"""Prepare identical JPEG-decoded inputs for the Python and native PnP paths."""
import argparse
from pathlib import Path
import sys
import cv2
import numpy as np
p=argparse.ArgumentParser()
p.add_argument('backend',type=Path)
p.add_argument('recording',type=Path)
p.add_argument('output',type=Path)
a=p.parse_args();sys.path.insert(0,str(a.backend.resolve()))
from tracking.odometry import RGBDOdometry
a.output.mkdir(parents=True,exist_ok=True)
file=cv2.FileStorage(str(a.output/'frames.yml'),cv2.FILE_STORAGE_WRITE)
file.startWriteStruct('frames',cv2.FileNode_SEQ)
expected=[];tracker=None
for index,path in enumerate(sorted(a.recording.glob('*.npz'))):
    with np.load(path) as f:
        rgb,depth,k,stamp=f['rgb'],f['depth'],f['K'],float(f['timestamp'])
    jpg=a.output/f'{index:05d}.jpg';raw=a.output/f'{index:05d}.f32'
    cv2.imwrite(str(jpg),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,75])
    depth.astype('<f4').tofile(raw)
    if tracker is None:tracker=RGBDOdometry(k,method='pnp')
    result=tracker.update(cv2.cvtColor(cv2.imread(str(jpg)),cv2.COLOR_BGR2RGB),depth,stamp)
    expected.append([{'initializing':0,'tracking':1,'lost':2}[result['status']],*result['pose'].flatten()])
    file.startWriteStruct('',cv2.FileNode_MAP)
    for key,value in [('image',str(jpg)),('depth',str(raw)),('stamp',stamp),('K',k)]:file.write(key,value)
    file.endWriteStruct()
file.endWriteStruct();file.release()
np.savetxt(a.output/'python.csv',expected,delimiter=',')
