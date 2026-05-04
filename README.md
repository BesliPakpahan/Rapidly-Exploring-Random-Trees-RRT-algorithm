# Rapidly-Exploring Random Trees (RRT) — Motion Planning Simulation

**SIMULASI RRT MOTION PLANNING**

**RE605 - Motion Planning**  
**Student NIM:** 4222301027

---

## Project summary
This repository contains an implementation of the RRT (Rapidly-Exploring Random Trees) sampling-based motion planning algorithm in Python. The simulation finds a collision-free path for a point robot from a Start position to a Goal position in a 2D workspace containing rectangular obstacles.

Files:
- `UTS/rrtuts.py` — RRT implementation, obstacle definitions, and visualization.
- `UTS/rrt_simulation_exact.png` — Example output visualization produced by the simulation.

---

## Environment and problem setup
- Workspace bounds: `x = -10..60`, `y = -10..60` (outer boundary walls included)
- Start: (0, 30)
- Goal: (50, 50)

Obstacles (rectangles given as `(x_min, y_min, x_max, y_max)`):

```
SIMULASI RRT MOTION PLANNING
RE605 - Motion Planning
Student NIM: 4222301027
======================================================================

OBSTACLE DISESUAIKAN DENGAN GAMBAR YANG DIBERIKAN

======================================================================
INFORMASI OBSTACLE
======================================================================
Total obstacle: 12

Detail setiap obstacle (x_min, y_min, x_max, y_max):
----------------------------------------------------------------------
Obstacle  1: ( -10.0,  -10.0,   -9.5,   60.0) - Size: 0.5 x 70.0
Obstacle  2: (  60.0,  -10.0,   60.5,   60.0) - Size: 0.5 x 70.0
Obstacle  3: ( -10.0,  -10.0,   62.0,   -9.5) - Size: 72.0 x 0.5
Obstacle  4: ( -10.0,   60.0,   62.0,   60.5) - Size: 72.0 x 0.5
Obstacle  5: ( -10.0,   40.0,   11.0,   40.5) - Size: 21.0 x 0.5
Obstacle  6: (  10.0,   20.0,   10.5,   40.0) - Size: 0.5 x 20.0
Obstacle  7: (  10.0,   20.0,   40.0,   20.5) - Size: 30.0 x 0.5
Obstacle  8: (  30.0,    5.0,   30.5,   20.0) - Size: 0.5 x 15.0
Obstacle  9: (  10.0,    5.0,   30.0,    5.5) - Size: 20.0 x 0.5
Obstacle 10: (  30.0,   40.0,   30.5,   60.0) - Size: 0.5 x 20.0
Obstacle 11: (  30.0,   40.0,   40.0,   40.5) - Size: 10.0 x 0.5
Obstacle 12: (  40.0,    5.0,   60.0,    5.5) - Size: 20.0 x 0.5
======================================================================

Menginisialisasi RRT Planner...

Menjalankan algoritma RRT...
Starting RRT planning...
Start: (0, 30)
Goal: (50, 50)
Iteration 500/5000, Nodes: 141
Iteration 1000/5000, Nodes: 250
Iteration 1500/5000, Nodes: 393
Goal reached in 1937 iterations!
Path length: 58 nodes
Path distance: 139.67 units

======================================================================
DETAIL JALUR YANG DITEMUKAN
======================================================================
Jumlah waypoints: 58
Total jarak jalur: 139.67 units
Jumlah node yang dieksplorasi: 541
Jarak garis lurus: 53.85 units
Efisiensi jalur: 38.6%

Waypoints jalur (10 pertama dan 10 terakhir):
----------------------------------------------------------------------
No     X            Y            Jarak Segmen        
----------------------------------------------------------------------
0      0.00         30.00        START               
1      -1.66        28.13        2.50                
2      -3.72        26.71        2.50                
3      -5.03        24.58        2.50                
4      -6.10        22.32        2.50                
5      -5.49        19.90        2.50                
6      -5.11        17.43        2.50                
7      -3.86        15.26        2.50                
8      -2.37        13.25        2.50                
9      -0.13        12.15        2.50                
...    ...          ...          ...                 
48     41.83        29.74        2.50                
49     42.76        32.06        2.50                
50     43.70        34.38        2.50                
51     44.63        36.70        2.50                
52     45.57        39.02        2.50                
53     46.50        41.33        2.50                
54     47.54        43.61        2.50                
55     48.44        45.94        2.50                
56     49.34        48.28        2.50                
57     50.00        50.00        1.85                 ← GOAL
======================================================================

Membuat visualisasi...

Visualization saved to: rrt_simulation_exact.png

✓ Simulasi berhasil!
✓ Jalur dari Start (0,30) ke Goal (50,50) ditemukan!
```

---

## How to run
1. Install requirements (if needed):

```bash
pip install numpy matplotlib
```

2. Run the simulation:

```bash
python3 UTS/rrtuts.py
```

This script prints obstacle info, planning progress, path details, and saves `UTS/rrt_simulation_exact.png`.

---

## Notes and next steps
- The RRT implementation is basic and works for this 2D rectangular obstacles environment.
- Improvements: smoothing (shortcutting), RRT* for optimal paths, KD-tree for faster nearest-neighbor queries.

---

## Visualization

![RRT Simulation](UTS/rrt_simulation_exact.png)

---

*Generated by the student code and assistant script.*
