# Dynamic obstacles for TEB

The publisher sends multiple moving obstacles to
`/move_base/TebLocalPlannerROS/obstacles` and publishes their bodies, complete
routes, velocity arrows, and short-term predictions to
`/prediction_to_teb/markers`.

```bash
roslaunch prediction_to_teb publish_dynamic_obstacle.launch
```

The default definitions are in `config/dynamic_obstacles.yaml`. Each obstacle
has a unique `id`, a `shape`, and a periodic `route`.

Supported shapes:

- `box`: `size: [width, depth, height]`; published to TEB as a four-point
  polygon and displayed in RViz as a cube.
- `circle`, `cylinder`, or `sphere`: `radius` and `height`; published to TEB as
  a circular obstacle.
- `polygon`: convex local `vertices` plus `height`; displayed as a prism.
- `line`: exactly two local `vertices`.

Supported routes:

- `square`: `center`, `size: [width, height]`, `speed`, optional `clockwise`
  and normalized `phase`.
- `circle`: `center`, `radius`, `speed`, optional `clockwise` and normalized
  `phase`.
- `waypoints`: `points`, `speed`, optional `closed` and normalized `phase`.
  An open route moves back and forth without a position jump.
- `stationary`: `position`.

Set `yaw_mode` to `tangent` to face the travel direction or `fixed` to use the
configured `yaw` in radians.

TEB is a 2D planner, so obstacle height is only visual. TEB predicts a dynamic
obstacle using the instantaneous linear velocity from each message. This node
publishes at 10 Hz by default, so velocity is updated as an obstacle turns a
corner or follows a curved route.

To use the legacy single circular-obstacle launch arguments:

```bash
roslaunch prediction_to_teb publish_dynamic_obstacle.launch use_config:=false
```
