include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint", -- 确保你的TF树里有这个frame，通常是机器人底盘中心
  published_frame = "odom",          -- Cartographer 发布 map -> odom
  odom_frame = "odom",               -- 里程计发布的 frame
  provide_odom_frame = false,        -- 如果你有外部里程计(odom->base_link)，这里设为 false
  publish_frame_projected_to_2d = true,
  use_odometry = true,               -- 必须开启，2D建图非常依赖里程计辅助
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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
MAP_BUILDER.use_trajectory_builder_2d = true
-- 建议使用的线程数，通常设置为 CPU 核心数 - 1
MAP_BUILDER.num_background_threads = 4 

-- ==================== 2. 前端参数 (Trajectory Builder) ====================

-- 激光雷达数据预处理
TRAJECTORY_BUILDER_2D.min_range = 0.3  -- 滤除车体自身的遮挡或极近噪点
TRAJECTORY_BUILDER_2D.max_range = 25.0 -- 大多数2D雷达的有效距离，太远的数据精度低
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 如果你有高精度IMU且TF正确，建议改为true

-- 自适应体素滤波 (Voxel Filter)，减少点云数量，提高匹配速度
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- *** 关键修改：Ceres 扫描匹配器 ***
-- 降低 translation_weight，允许算法更多地依赖激光雷达数据来修正里程计的误差
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- *** 关键修改：实时相关性扫描匹配 (CSM) ***
-- 如果里程计比较准，可以减小搜索窗口以节省计算资源
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.

-- *** 关键修改：运动过滤器 (Motion Filter) ***
-- 避免在机器人静止或微小震动时不断插入新子图
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5.0) -- 从 1度 改为 5度

-- 子图大小，通常 90-100 是标准值
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- ==================== 3. 后端参数 (Pose Graph / 闭环) ====================

-- *** 关键修改：优化频率 ***
-- 30 太频繁了，改为 90 或更高，避免建图过程中机器卡顿
POSE_GRAPH.optimize_every_n_nodes = 60

-- *** 关键修改：闭环检测阈值 ***
-- 提高阈值，宁可少闭环，也不要错误闭环。0.65 是一个比较安全的工业标准值。
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 采样比率：只计算 30% 的节点约束，大幅降低 CPU 占用
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- 限制闭环搜索的距离，防止和地图另一端相似的走廊发生错误匹配
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 闭环扫描匹配器参数
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- 最后的全局优化参数
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e1 -- 如果机器人加减速很猛，可以调大
POSE_GRAPH.optimization_problem.rotation_weight = 3e5

return options