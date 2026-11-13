#!/bin/bash
# record one stud-top touch in teach_plate
cd "$(dirname "$0")"
n0=$(grep -c "stud-top touch" teach.log); echo z >> teach.in
timeout 4 bash -c "until [ \$(grep -c 'stud-top touch' teach.log) -gt $n0 ]; do sleep 0.1; done"
grep "stud-top touch" teach.log | tail -1 | sed 's/^> *//'
