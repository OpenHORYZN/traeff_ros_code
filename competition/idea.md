### Idea

1. ROS2 NODE
2. INPUT: nav_msgs/msg/Path.msg = {p1, p2, p3, ...}
3. OUTPUT: current velocity geometry_msgs/TwistStamped

*KEY CONCEPT*: Time warping (preserve positions, scale time)

### EXAMPLE

**Step 1: Original Path**
Waypoints (x, y):
- (1, 2)
- (2, 3)  
- (4, 5)

**Step 2: Arc Length Parameterization**

Compute cumulative distances:
- d0 = 0
- d1 = distance(p0→p1) = √[(2-1)² + (3-2)²] = √2 ≈ 1.414
- d2 = d1 + distance(p1→p2) = 1.414 + √[(4-2)² + (5-3)²] = 1.414 + √8 ≈ 4.242

Interpolate position as function of arc length s:
- x(s) = cubic spline through (d0, x0), (d1, x1), (d2, x2)
- y(s) = cubic spline through (d0, y0), (d1, y1), (d2, y2)

**Step 3: Find Peak Derivative**

Derivative w.r.t arc length: f'(s) = [dx/ds, dy/ds]

Sample f'(s) along path to find peak magnitude:
- V'_max = max ||f'(s)|| over s ∈ [0, d_total]

Example values:
- At s=0: f' = [0.8, 0.6] → ||f'|| = 1.0
- At s=2: f' = [1.5, 1.2] → ||f'|| = 1.92
- At s=4: f' = [0.5, 0.3] → ||f'|| = 0.58

V'_max = 1.92

**Step 4: Time Scaling Factor**

Given V_max = 3 m/s (velocity limit)

If V'_max > V_max:
    α = V'_max / V_max  = 1.92 / 3 = 0.64
Else:
    α = 1

Wait — correction: We want to scale time, not velocity.
Actual logic:

Original time if moving at 1 m/s in arc length: T_original = d_total = 4.242 s

To limit velocity to V_max:
- Required: ||dp/dt|| = ||dp/ds|| * |ds/dt| ≤ V_max
- Choose |ds/dt| = V_max / V'_max = 3 / 1.92 ≈ 1.5625

Therefore:
- New total time = d_total / |ds/dt| = 4.242 / 1.5625 ≈ 2.715 s
- Time scaling factor = T_new / T_original = 0.64

**Step 5: Generate Velocity Commands**

Set control period Δt = 50 ms

For i = 0, 1, 2, ... until T_new:
    t = i × Δt
    
    # Map time to arc length (constant speed in arc length)
    s = (t / T_new) × d_total
    
    # Get desired velocity (derivative w.r.t time)
    V_i = f'(s) × (ds/dt) = f'(s) × (d_total / T_new)
    
    where ds/dt = d_total / T_new = 4.242 / 2.715 ≈ 1.5625 m/s

Result: V_i magnitude never exceeds V_max = 3 m/s

**Step 6: Publish**

Create publisher to publish current velocities as geometry_msgs/TwistStamped at each Δt

### Alternative: Simple Scaling (Your Original)

If you MUST scale the function directly (not recommended because it changes positions):

1. Find V'_max = max |f'(x)| on [x_start, x_end]
2. Compute k = V_max / V'_max
3. Create g'(x) = k × f'(x)
4. Integrate to get g(x) = k × f(x) + C
5. Adjust C so g(x_start) = f(x_start) and g(x_end) = f(x_end) — but this is impossible unless f(x_start)=0

**Why this fails:** Scaling the derivative changes total displacement unless you reparameterize, which is exactly what time warping does properly.

### Summary

| Approach | Preserves waypoints? | Limits velocity? | Method |
|----------|---------------------|------------------|---------|
| Function scaling | ❌ No | ✅ Yes | Multiply f'(x) by k |
| Time warping | ✅ Yes | ✅ Yes | Scale time, keep positions |

**Use time warping** for correct trajectory following with velocity limits.