#!/bin/bash
# record one grip in teach_plate: rec_cell.sh a b
cd "$(dirname "$0")"
n0=$(grep -c "cell ($1,$2)" teach.log); echo "$1 $2" >> teach.in
timeout 4 bash -c "until [ \$(grep -c 'cell ($1,$2)' teach.log) -gt $n0 ]; do sleep 0.1; done"
grep "cell ($1,$2)\|WARNING: gripper" teach.log | tail -1 | sed 's/^> *//'
