# 417FinalReportMyVersion

Sample run experiment:
let the current directory be 'final'

python run_experiments.py --instance instances/high_level_12x12.txt --solver IDCBS

IMPORTANT FILES:
IDCBS.py high-level
epea_star.py (Amy's implement)
EPEIDE.py low-level 

Setup:
Operating System: MacOS Catalina Version 10.15.7
Programming Language: Python 3.8.5
Processor: 2.5 GHz Dual-Core Intel Core i5
Memory: 16 GB 1600 MHz DDR3

run tests:
python run_experiments.py --instance instances/high_level_7x7.txt --solver IDCBS
CPU time (s):    0.02
Sum of costs:    11
Expanded nodes:  9
Generated nodes: 16
Low Level nodes: 109

python run_experiments.py --instance instances/high_level_8x8.txt --solver IDCBS
CPU time (s):    0.03
Sum of costs:    15
Expanded nodes:  14
Generated nodes: 26
Low Level nodes: 224

python run_experiments.py --instance instances/high_level_9x9.txt --solver IDCBS
CPU time (s):    0.08
Sum of costs:    19
Expanded nodes:  49
Generated nodes: 95
Low Level nodes: 940

python run_experiments.py --instance instances/high_level_10x10.txt --solver IDCBS
CPU time (s):    0.09
Sum of costs:    23
Expanded nodes:  29
Generated nodes: 56
Low Level nodes: 679

python run_experiments.py --instance instances/high_level_11x11.txt --solver IDCBS
CPU time (s):    0.11
Sum of costs:    27
Expanded nodes:  37
Generated nodes: 72
Low Level nodes: 1013

python run_experiments.py --instance instances/high_level_12x12.txt --solver IDCBS
CPU time (s):    0.13
Sum of costs:    31
Expanded nodes:  46
Generated nodes: 90
Low Level nodes: 1442
