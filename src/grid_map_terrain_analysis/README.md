# grid_map_terrain_analysis

ROS 2 terrain analysis node based on the apt-installed ANYbotics `grid_map`
packages. It keeps a rolling local grid map in the `map` frame, analyzes terrain
from registered point clouds, and publishes a `/terrain_map` point cloud that is
compatible with the current `local_planner` input convention. The planner-facing
intensity is a continuous risk value, not a binary obstacle label.

## Inputs

- `/registered_scan` (`sensor_msgs/msg/PointCloud2`)
  - Required frame: `map`
  - Fields used: `x`, `y`, `z`
- `/state_estimation` (`nav_msgs/msg/Odometry`)
  - Required frame: `map`
  - Used for rolling-map center, robot-relative height filtering and
    dynamic-obstacle cache alignment

The node does not perform TF transforms. The input cloud and odometry must
already be expressed in `map`.

## Outputs

- `/terrain_map` (`sensor_msgs/msg/PointCloud2`)
  - Frame: `map`
  - Point type: `x`, `y`, `z`, `intensity`
  - `x`, `y`, `z`: filtered input cloud point position
  - `intensity`: planner risk probability, `1 - P(traversable)`
    - `0.0`: fully traversable
    - `1.0`: not traversable / obstacle
    - intermediate values represent partial traversability from height, slope,
      roughness, step and dynamic-obstacle evidence
    - This keeps compatibility with `local_planner`, which treats larger
      `intensity` values as more blocking.
- `/elevation_map` (`grid_map_msgs/msg/GridMap`)
  - Debug layers: `elevation`, `max_height`, `height_gap`, `density`
- `/traversability_map` (`grid_map_msgs/msg/GridMap`)
  - Debug layers: `elevation`, `elevation_inpainted`, `slope`, `roughness`,
    `step_height`, `height_risk`, `slope_risk`, `roughness_risk`, `step_risk`,
    `density_confidence`, `terrain_risk`, `dynamic_risk`, `final_risk`,
    `traversability`, `risk`, `static_obstacle`, `dynamic_obstacle`,
    `intensity`
- `/elevation_map_cloud` (`sensor_msgs/msg/PointCloud2`)
  - RViz-friendly copy of the elevation layer
  - `z`: grid cell elevation
  - `intensity`: elevation value
- `/traversability_map_cloud` (`sensor_msgs/msg/PointCloud2`)
  - RViz-friendly traversability probability map
  - `z`: inpainted grid elevation
  - `intensity`: pure traversability probability, `P(traversable)`
    - `0.0`: not traversable
    - `1.0`: fully traversable
- `/terrain_risk_cloud` (`sensor_msgs/msg/PointCloud2`)
  - `intensity`: static terrain risk before dynamic-obstacle fusion
- `/dynamic_obstacle_cloud` (`sensor_msgs/msg/PointCloud2`)
  - `intensity`: continuous dynamic-obstacle risk from the world-frame cache
- `/terrain_height_cloud` (`sensor_msgs/msg/PointCloud2`)
  - `intensity`: maximum ground-relative height gap per cell

## Build

```bash
cd /home/neepu/kaiyuan_pkg/grid_map
source /opt/ros/humble/setup.bash
colcon build --packages-select grid_map_terrain_analysis \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

```bash
source /home/neepu/kaiyuan_pkg/grid_map/install/setup.bash
ros2 launch grid_map_terrain_analysis grid_map_terrain.launch.py
```

## Main Parameters

Edit `config/terrain_params.yaml`.

- `map_length_x`, `map_length_y`: rolling local map size in meters
- `resolution`: grid resolution in meters
- `min_rel_z`, `max_rel_z`: robot-relative height filter for raw cloud points
- `dis_ratio_z`: distance-based relaxation for the robot-relative height filter
- `min_ground_relative_height`, `max_ground_relative_height`: final
  ground-relative height window for `/terrain_map`; this filters ceiling and
  high suspended points after ground elevation is estimated
- `elevation_quantile`: low quantile used as local ground estimate
- `height_safe`, `height_block`: height-gap range mapped to continuous risk
- `slope_safe`, `slope_block`: slope range mapped to continuous risk, in radians
- `roughness_safe`, `roughness_block`: roughness range mapped to continuous risk
- `step_safe`, `step_block`: local height-step range mapped to continuous risk
- `unknown_traversability`: fallback traversability for low-confidence cells
- `min_density_for_confidence`: point density needed for full observation
  confidence
- `risk_gamma`: output curve shaping for final risk; values below `1.0`
  make risk more sensitive, values above `1.0` make it smoother
- `dynamic_enabled`: enable world-frame dynamic-obstacle risk memory
- `dynamic_hit_gain`: score gain from height-risk evidence
- `dynamic_free_decay`: decay applied when a cell is observed free
- `dynamic_time_decay`, `dynamic_decay_time`: time decay base and interval
- `dynamic_keep_time`: remove old dynamic cells after this time when score is
  near zero
- `dynamic_min_height_risk`: minimum height risk that contributes dynamic hits
- `publish_only_blocking_points`: publish only points whose intensity can affect
  the planner. The default is `false` to keep the output close to the input
  point-cloud terrain map style
- `blocking_intensity_threshold`: minimum planner risk kept in `/terrain_map`
  when `publish_only_blocking_points` is enabled

## Output Shape

`/terrain_map` is not generated as one point per grid cell. The node uses
`grid_map` to calculate terrain layers, then iterates over the filtered input
cloud and annotates each point with the terrain result from its grid cell. This
keeps the output close to the original `terrain_analysis` style: point positions
come from the cloud, while `intensity` carries planner risk probability
`1 - P(traversable)`.

## Risk Model

The static terrain risk is continuous:

```text
height_risk    = smoothstep(height_safe, height_block, height_gap)
slope_risk     = smoothstep(slope_safe, slope_block, slope)
roughness_risk = smoothstep(roughness_safe, roughness_block, roughness)
step_risk      = smoothstep(step_safe, step_block, step_height)

P_terrain =
  (1 - height_risk) *
  (1 - slope_risk) *
  (1 - roughness_risk) *
  (1 - step_risk)

terrain_risk = 1 - P_terrain
```

Low-density cells blend toward `unknown_traversability` instead of becoming
hard free or hard blocked.

Dynamic obstacles are stored in a world-coordinate cell cache keyed by map
position, so the risk memory does not drift when the rolling grid map moves with
the robot. Dynamic risk rises from repeated height-risk evidence and decays when
the same world cell is seen free or ages out.

The final planner risk is:

```text
final_risk = 1 - (1 - terrain_risk) * (1 - dynamic_risk)
intensity = pow(final_risk, risk_gamma)
```

## Dependency Choice

This package uses the installed ROS 2 Humble grid_map packages from
`/opt/ros/humble`. The local `grid_map` source tree is not modified.
