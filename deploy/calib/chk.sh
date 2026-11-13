#!/bin/bash
# status + finger closing direction: chk.sh
cd "$(dirname "$0")"
echo status >> gripper.ctl; sleep 0.4; L=$(tail -1 gripper.log); echo "$L"
Q=$(echo "$L" | sed -E 's/^q \[([^]]*)\].*/\1/')
cd .. && OPENBLAS_NUM_THREADS=1 ~/alphalego-venv/bin/python -c "
import numpy as np, core
p,R=core.tcp_pose(np.radians([$Q]),0.1425); n=R[:,1].copy(); n[2]=0
print('closing along', np.round(n[:2]/np.linalg.norm(n[:2]),2), ' offset from SIM cell 0 0 (166, 41):', np.round(p[:2]*1e3-[166.0,41.0],1))"
