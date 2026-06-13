include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint", -- IMU 和 激光雷达 必须通过 TF 连接到这里
  published_frame = "odom",          -- Cartographer 发布 map -> odom
  odom_frame = "odom",
  provide_odom_frame = false,        -- 外部里程计提供 odom -> base_link
  publish_frame_projected_to_2d = false, -- 3D 建图通常不需要强制投影到 2D
  use_odometry = true,               -- 依然建议开启，提供速度约束
  use_nav_sat = false,
  use_landmarks = false,
  
  -- *** 关键修改：输入源切换为 3D 点云 ***
  num_laser_scans = 0,               -- 2D 雷达设为 0
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,              -- 开启 3D 雷达输入
  
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ==================== 1. Map Builder (后端算法选择) ====================
-- *** 关键修改：启用 3D 轨迹构建器 ***
MAP_BUILDER.use_trajectory_builder_3d = true
-- 3D 建图计算量大，建议尽可能多分配线程 (CPU 核心数 - 1)
MAP_BUILDER.num_background_threads = 7 

-- ==================== 2. 前端参数 (Trajectory Builder 3D) ====================

-- *** 关键修改：点云滤波与范围 ***
-- 3D雷达数据量大，必须进行体素滤波以降低计算压力
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 60.0 -- 3D 雷达看的一般比 2D 远
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1 -- 如果雷达频率高（如10Hz），设为1；如果需要累积多帧增加密度，可设为2-3

-- 高分辨率体素滤波：用于近距离的高精度匹配
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.0

-- 低分辨率体素滤波：用于远距离和闭环检测（减少计算量）
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 60.0

-- *** 关键修改：IMU 使用 ***
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false -- 3D 中通常关闭 CSM，太慢了，主要靠 Ceres + IMU
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 120

-- Ceres 扫描匹配器 (前端里程计核心)
-- 3D 需要优化 6 自由度，不再只是平移+Yaw
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 40. -- 强依赖 IMU 提供初值，但也允许点云修正
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false -- 必须为 false，允许优化 Roll/Pitch

-- *** 关键修改：子图生成 ***
-- 3D 子图需要更多的点才能稳定，增加 range_data 数量
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10 -- 地图分辨率 10cm
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45  -- 远距离用低分辨率
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160   -- 增加帧数以构建更稠密的子图

-- 运动过滤器 (Motion Filter)
-- 保持和你 2D 类似的逻辑，避免静止时建图
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(5.0)

-- ==================== 3. 后端参数 (Pose Graph / 闭环) ====================

-- 优化频率
POSE_GRAPH.optimize_every_n_nodes = 80 -- 3D 节点更重，适当减少优化频率避免卡顿

-- *** 关键修改：闭环检测阈值 ***
-- 3D 匹配比 2D 难，分数通常较低。0.55 是 3D 中常用的保守值。
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- 限制闭环距离
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 闭环匹配器参数 (Fast Correlative Scan Matcher 3D)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.)

-- Ceres 优化权重
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e1
POSE_GRAPH.optimization_problem.rotation_weight = 3e5

-- 3D 特有：防止地图在 Z 轴（高度）上漂移，依赖 IMU 重力对齐
POSE_GRAPH.optimization_problem.fix_z_in_3d = false -- 如果是在完全平面上跑可以设为 true，但在坡道上必须为 false
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true -- 允许在线微调 IMU 外参

return options