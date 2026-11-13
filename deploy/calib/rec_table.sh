#!/bin/bash
# record one table spot: teach_plate 'table' + a gripper-tool status line, appended to calib/table_series.txt
cd "$(dirname "$0")"
n0=$(grep -c "tabletop:" teach.log); echo table >> teach.in; echo status >> gripper.ctl
timeout 4 bash -c "until [ \$(grep -c 'tabletop:' teach.log) -gt $n0 ]; do sleep 0.1; done"
t=$(grep "tabletop:" teach.log | tail -1 | sed 's/^> *//'); s=$(grep "^q " gripper.log | tail -1)
q=$(echo "$s" | sed -E 's/^q \[([^]]*)\].*/\1/'); tcp=$(echo "$s" | sed -E 's/.*tcp \(([^)]*)\).*/\1/')
k=$(( $(wc -l < table_series.txt 2>/dev/null || echo 0) + 1 ))
echo "spot $k | tcp ($tcp) mm | q [$q] | $t" | tee -a table_series.txt
