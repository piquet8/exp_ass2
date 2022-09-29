Number of literals: 11
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 120.001)b (8.000 | 120.002)b (7.000 | 180.003)b (6.000 | 180.004)b (5.000 | 240.005)b (4.000 | 240.006)b (3.000 | 300.007)b (2.000 | 360.008)b (1.000 | 360.008);;;; Solution Found
; States evaluated: 18
; Cost: 420.009
; Time 0.01
0.000: (moveto_hint wphome wp1)  [60.000]
60.001: (take_hint wp1)  [60.000]
60.002: (goto_waypoint wp1 wp2)  [60.000]
120.003: (take_hint wp2)  [60.000]
120.004: (goto_waypoint wp2 wp3)  [60.000]
180.005: (take_hint wp3)  [60.000]
180.006: (goto_waypoint wp3 wp4)  [60.000]
240.007: (take_hint wp4)  [60.000]
240.008: (moveto_oracle wp4 wphome)  [60.000]
300.008: (hyp_ready wphome)  [60.000]
360.009: (check_hyp wphome)  [60.000]
