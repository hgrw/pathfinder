# pathfinder

AGENT
- The agent (A) is represented as a line segment of length w unit, where w ∈ [0.05, 0.15]
- 3 DOF (x, y, theta at centroid)
- Pushes one object at a time
- 75% of edge must be in contact with movable object to initiate move
- Primitive actions quantised to 0.001 units max
- The agent can move and turn at the same time, but the travel distance of both endpoints of the robot should be less than the step size.

ENVIRONMENT
- The robot operates in a 2D environment of size [0, 1] X [0, 1], populated with
- Axis aligned moving boxes, (B) of size w x w
- Axis aligned moving square obstacles (O_m) with width in range(1, 1.5w)
- Axis aligned static rectangular obstacles (O_s)

TASK
- Each moving box, B, has a goal location, G (mutually exclusive?)
- Collision free path to be generated for each moving box. I.e. A, B, O_m, O_s, do not collide at any point

INPUT FILE
- Line 1, agent parameters:                 w, x, y, theta (radians)
- Line 2, objects:                    num(B), num(O_m), num(O_s)
- Line  i ∈ [3, m+2], initial & goal positions:        (x, y)_start, (x, y)_goal
- Line  i ∈ [m+3, m+n+2], O_m pos & size:        (x, y)_O_m, width
- Line  i ∈ [m+n+3, m+n+o+2], O_s pos & geometry:    (x, y)_lower_left, (x, y)_upper_right

OUTPUT FILE
- Path definition. Each line defines a primitive step.
- Line 1, number of steps:                p
- Line i ∈ [2, p], path definition at timestep:        x, y, theta, (x, y)_O_m, …, (x, y)_O_s, ...