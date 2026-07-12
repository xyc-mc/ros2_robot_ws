# scan_traversability_filter

Subscribes to `/registered_scan` and `/elevation_mapping_node/elevation_map_raw`,
looks up each scan point's `x/y` in the GridMap `traversability` layer, and publishes
`/terrain_map` as `sensor_msgs/msg/PointCloud2`.

Output fields:

- `x/y/z`: copied from `/registered_scan`
- `intensity`: `1.0 - traversability`

## Height filtering

Before looking up traversability, the node rejects points above the same
sensor-relative height envelope used by `elevation_mapping_cupy`:

```text
relative_z = point.z - reference.z
dxy = max(horizontal_distance(point, reference) - ramped_height_range_b, 0)
relative_z <= max_height_range
relative_z <= dxy * ramped_height_range_a + ramped_height_range_c
```

The reference pose is obtained from TF at the scan timestamp. The default
reference frame is `body`, matching the current elevation mapping input
`/cloud_registered_body`. If the transform is unavailable, the complete scan
is dropped so unfiltered ceiling points cannot reach `/terrain_map`.

The launch defaults currently match
`elevation_mapping_cupy/config/core/core_param.yaml`:

- `max_height_range: 0.3`
- `ramped_height_range_a: 0.5`
- `ramped_height_range_b: 0.8`
- `ramped_height_range_c: 0.35`

Keep these four launch values synchronized when changing the elevation mapping
height envelope.

Run:

```bash
ros2 launch scan_traversability_filter scan_traversability_filter.launch.py
```
