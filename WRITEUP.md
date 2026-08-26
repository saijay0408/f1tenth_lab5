# Lab 5 (Optional): Scan Matching

This is a Python reimplementation of the scan-matching concept taught in this lab, built for
this project's ROS2/`f1tenth_gym_ros` simulator pipeline. The official lab skeleton requires
C++ and a ROS1 simulator stack (`racecar_simulator`) as a group assignment; this submission
instead implements the same core algorithm (ICP-based scan-matching localization) as an
individual, Python, ROS2 node, verified against the real simulator.

## Approach

`scan_matcher.py` estimates the robot's pose purely from consecutive LiDAR scans:

1. Each `LaserScan` is converted to Cartesian points in the sensor frame, downsampled and
   range-filtered.
2. Point-to-point ICP aligns the current scan against the previous scan: nearest-neighbor
   correspondences are found (with outlier rejection beyond `max_corr_dist`), then the optimal
   rigid transform between the matched points is solved in closed form via SVD (the
   Kabsch/Procrustes algorithm), iterated until convergence.
3. The resulting per-step transform is composed into a running global pose estimate and
   published as `nav_msgs/Odometry` on `/scan_match_odom`.

## Theoretical Questions

**1a. Show B_i := M_i^T M_i is symmetric.**

For any matrix M_i, (M_i^T M_i)^T = M_i^T (M_i^T)^T = M_i^T M_i. So B_i^T = B_i by definition.

**1b. Show B_i is positive semi-definite.**

For any vector v: v^T B_i v = v^T M_i^T M_i v = (M_i v)^T (M_i v) = ||M_i v||^2 >= 0.
Since v^T B_i v >= 0 for all v, B_i is positive semi-definite (it is a Gram matrix).

**2a. Find M, W, g.**

Expanding the objective:

||M_i x - pi_i||^2 = x^T M_i^T M_i x - 2 pi_i^T M_i x + pi_i^T pi_i

Summing over i and dropping the constant term (it doesn't affect the argmin):

x* = argmin_x  x^T (sum_i M_i^T M_i) x  -  2 (sum_i pi_i^T M_i) x

So: **M = sum_i M_i^T M_i**, **g = -2 sum_i M_i^T pi_i**.

The constraint x3^2 + x4^2 = 1 in quadratic form x^T W x = 1 is exactly:

**W = diag(0, 0, 1, 1)**

**2b. Show M and W are positive semi-definite.**

M = sum_i M_i^T M_i = sum_i B_i, and each B_i is PSD (from 1b). A sum of PSD matrices is PSD,
since for any v: v^T M v = sum_i v^T B_i v = sum_i ||M_i v||^2 >= 0.

W = diag(0,0,1,1) is diagonal with non-negative entries, so for any v = (v1,v2,v3,v4):
v^T W v = v3^2 + v4^2 >= 0. Hence W is PSD.

## Performance Analysis

Evaluated against the real simulator: `wall_follow` (Lab 3) drove the car continuously around
the Levine map while `scan_matcher` ran independently, comparing its scan-matched position
estimate against the simulator's ground-truth `/ego_racecar/odom`, both measured as relative
displacement from a common starting point.

**Two real issues were found and fixed during testing:**

1. **Stale build artifact.** Early test runs kept reproducing identical, frozen trajectories
   regardless of code changes. The installed package in the test container had gone stale after
   a `docker cp` that silently didn't take effect (a repeated `rm -rf` of the install/build
   directories before every copy fixed it permanently). All of the earlier "gets stuck" results
   were actually testing old code, not the current algorithm.
2. **Match rate too high for the signal.** Once testing against the real (fresh) build, the
   `/scan` topic in this simulator publishes at a very high rate. Matching every single scan
   meant each ICP step's true motion was sub-millimeter -- a poor signal-to-noise ratio that let
   tiny per-step correspondence/rounding bias compound over thousands of steps into meters of
   drift. Throttling scan matching to a ~10Hz cadence (so each step has a meaningful amount of
   real motion, e.g. ~0.1-0.2m at driving speed) cut the mean tracking error roughly in half.

**Final measured performance** (25s continuous drive around the Levine loop, corners included):

- Mean position error: ~3.65m
- Best-case tracking: as low as 0.20m during a well-conditioned straight/curve segment
- Worst-case: ~8.8m, concentrated around sharp corners

**Limitations:** point-to-point ICP (rather than the point-to-line/PLICP metric from the Censi
paper this lab is based on) is known to drift along features with poor lateral constraint, such
as long straight walls, since points can "slide" along the wall without increasing the ICP
error. This shows up here as the largest drift occurring around corners, where the correspondence
geometry changes quickly between scans. There is also no loop closure or global correction --
this is pure incremental dead-reckoning, so error is expected to grow unbounded over a long run
even when short-term tracking is accurate. A point-to-line metric, tighter correspondence
rejection tuned per-corner, or fusing with the true odometry/IMU would all be natural next steps
to reduce this drift.
