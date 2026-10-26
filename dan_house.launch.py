#!/usr/bin/env python3
"""
dan_house.launch.py —— Gazebo Fortress (gz-sim 6) + Lighthouse（灯塔）+ SLAM/Nav2 一体化启动

用法（系统终端，先 source /opt/ros/humble/setup.bash，不要在 .venv 里跑）:

  1) 只看仿真 + 机器人
     ros2 launch ./dan_house.launch.py rviz:=true

  2) SLAM 建图（slam_toolbox 在线异步）
     ros2 launch ./dan_house.launch.py slam:=true rviz:=true

  3) Nav2 自主导航（用之前存好的地图）
     ros2 launch ./dan_house.launch.py nav:=true rviz:=true

  4) 无界面（ssh / CI）
     ros2 launch ./dan_house.launch.py headless:=true slam:=true

参数：
  slam:=true|false       启动 slam_toolbox
  nav:=true|false        启动 nav2_bringup（定位+导航，需要先有地图）
  map:=<path.yaml>       导航地图，默认 config/maps/dan_house.yaml
  rviz:=true|false       启动 RViz（带预配置的 dan_house.rviz）
  headless:=true|false   只起 gz server，不开 GUI
  use_sim_time:=true     默认 true
  lidar:=2d|3d           雷达版本（默认 2d，行为与以前完全一致）
     2d → worlds/dan_house.world    + models/lighthouse（单线，桥接 /scan LaserScan）
     3d → worlds/dan_house_3d.world + models/lighthouse_3d（64 线点云，
          桥接 /scan/points PointCloud2，再由 pointcloud_to_laserscan 投影出 /scan）

用法补充：
  5) 64 线雷达版建图
     python3 build_world.py --lidar 3d        # 只需在改过世界后跑一次
     ros2 launch ./dan_house.launch.py lidar:=3d slam:=true rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------- 两个机器人版本共用一套世界，只换 include 的模型 ----------
# 2d：单线雷达（models/lighthouse）        —— 现有路径，完全不动
# 3d：64 线雷达（models/lighthouse_3d）    —— 点云 + 投影
# 世界文件由 build_world.py 生成：
#   python3 build_world.py              → worlds/dan_house.world
#   python3 build_world.py --lidar 3d   → worlds/dan_house_3d.world
MODELS_DIR = os.path.join(HERE, 'models')
LH_MODEL_2D = os.path.join(MODELS_DIR, 'lighthouse')
LH_MODEL_3D = os.path.join(MODELS_DIR, 'lighthouse_3d')
LH_URDF_2D = os.path.join(LH_MODEL_2D, 'robot.urdf')
LH_URDF_3D = os.path.join(LH_MODEL_3D, 'robot.urdf')
WORLD_2D = os.path.join(HERE, 'worlds', 'dan_house.world')
WORLD_3D = os.path.join(HERE, 'worlds', 'dan_house_3d.world')
ODOM_TF = os.path.join(HERE, 'odom_tf_node.py')
SLAM_PARAMS = os.path.join(HERE, 'config', 'slam_toolbox.yaml')
NAV2_PARAMS = os.path.join(HERE, 'config', 'nav2_params.yaml')
# 导航专用参数（速度快）：建图/探索用上面那份（0.15 m/s，图才稳），
# 图建好后做定位导航用这份（0.26 m/s）。
NAV2_PARAMS_NAV = os.path.join(HERE, 'config', 'nav2_params_nav.yaml')
# 探索专用 Nav2 参数：与 nav2_params.yaml 只差两处 —— BT 里 BackUp 排第一、
# 进度判定改成 0.25 m / 6 s。只在 explore:=true 时生效，验收走 nav2_params*.yaml。
NAV2_PARAMS_EXPLORE = os.path.join(HERE, 'config', 'nav2_params_explore.yaml')
DEFAULT_MAP = os.path.join(HERE, 'config', 'maps', 'dan_house.yaml')
RVIZ_CFG = os.path.join(HERE, 'rviz', 'dan_house.rviz')
TOUR_NODE = os.path.join(HERE, 'tour_node.py')
FRONTIER_NODE = os.path.join(HERE, 'frontier_explore.py')
EKF_PARAMS = os.path.join(HERE, 'config', 'ekf.yaml')

# odom_tf_node.py 用哪个 python 跑？
# ROS Humble 的 rclpy 是给系统 python3.10 编译的，所以这里写死 /usr/bin/python3，
# 不要用 PATH 里的 python3（conda env / pyenv / 各种 venv 都会导致
# ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'）。
ROS_PYTHON = os.environ.get('ROS_PYTHON', '/usr/bin/python3')
if not os.path.exists(ROS_PYTHON):
    ROS_PYTHON = 'python3'

# 机器人 URDF（TF 链 + RViz 外形）—— 两台车各一份，由 lidar:=2d|3d 选（常量见文件头）：
#   2d → models/lighthouse/robot.urdf      （单线，base_scan 外形 = lds.stl）
#   3d → models/lighthouse_3d/robot.urdf   （64 线，base_scan 外形 = Ø90×90 雷达头）
# 血缘：结构照抄官方 turtlebot3_waffle.urdf（关节树/轮子/IMU 参数），
# 但改掉了雷达与相机的位置、补上了塔身，网格也复制进各自 models/<robot>/meshes/。
# 为什么必须自己维护：TF 树只认 URDF，SDF 里的 <pose> 只影响 Gazebo 的物理/渲染。


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    # ---------- 雷达版本开关：一个参数决定世界/模型/桥接/投影 ----------
    # 用 PythonExpression 做三元选择，这样 lidar:=2d 时算出来的值和改动前逐字相同。
    lidar = LaunchConfiguration('lidar')
    lidar_is_3d = PythonExpression(["'", lidar, "' == '3d'"])
    world = PythonExpression([
        "'", WORLD_3D, "' if '", lidar, "' == '3d' else '", WORLD_2D, "'"])
    lh_model = PythonExpression([
        "'", LH_MODEL_3D, "' if '", lidar, "' == '3d' else '", LH_MODEL_2D, "'"])
    lh_urdf = PythonExpression([
        "'", LH_URDF_3D, "' if '", lidar, "' == '3d' else '", LH_URDF_2D, "'"])

    # ---------- 0) 可选：把渲染丢给独立显卡 ----------
    # 这台机器是双显卡（Intel 核显 + RTX 4080），Gazebo 默认只走核显。
    # 用 nvidia:=true 启动即可走 PRIME 渲染卸载；不想要就保持默认 false。
    use_nvidia = IfCondition(LaunchConfiguration('nvidia'))
    offload_to_nvidia = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1', condition=use_nvidia),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia', condition=use_nvidia),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only', condition=use_nvidia),
        # Lighthouse 的 SDF 里网格写成 model://lighthouse/meshes/...，
        # 靠这个环境变量解析；把 models/ 目录加进去（保留系统原有的值）。
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(HERE, 'models') + (
                ':' + os.environ['GZ_SIM_RESOURCE_PATH']
                if os.environ.get('GZ_SIM_RESOURCE_PATH') else '')),
    ]

    # ---------- 1) Gazebo（GUI 模式 / 纯 server 模式二选一） ----------
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -v 3 ', world]}.items(),
        condition=UnlessCondition(headless),
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-s -r -v 3 ', world]}.items(),
        condition=IfCondition(headless),
    )

    # ---------- 2) gz -> ROS 话题桥 ----------
    #  [  = gz -> ROS ;  ] = ROS -> gz ;  @ = 双向
    #
    # lidar:=2d → 桥 gz 的 /scan（LaserScan），与改动前完全一致
    # lidar:=3d → 只桥 /scan/points（PointCloudPacked），/scan 交给投影节点生成
    #   ⚠️ 3d 时绝不能同时桥 gz 的 /scan：gpu_lidar 在多线模式下会额外发一条
    #      单圈 LaserScan，桥过来就与投影节点的 /scan 撞成两个 publisher，
    #      SLAM 会收到混在一起的两圈数据。
    BRIDGE_COMMON = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/odom_wheels@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ]
    bridge_2d = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        arguments=BRIDGE_COMMON + [
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(lidar_is_3d),
    )
    bridge_3d = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        arguments=BRIDGE_COMMON + [
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(lidar_is_3d),
    )

    # ---------- 2b) 点云 → 2D 扫描投影（仅 lidar:=3d） ----------
    # 64 线雷达给的是 3D 点云，而 slam_toolbox / Nav2 / 智能体层吃的都是 /scan。
    # 这个节点把高度带 [-0.75, +0.05] m（相对 base_scan）内的点压成 360 格的一圈距离：
    #   -0.75 m → 地面以上 0.03 m（base_scan 在 0.78）
    #   +0.05 m → 0.83 m，正好不把雷达头自己的顶盖（+0.053）算进来
    # 所以茶几 0.40、垃圾桶 0.63、马桶这些低矮家具都会进入 /scan —— 单线雷达做不到。
    # range 与 slam_toolbox 的 min/max_laser_range（0.12 / 12.0）严格对齐；
    # 角度 π/180 = 360 格，与单线雷达的角度分辨率一致 → 下游参数一个都不用改。
    projector = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/scan/points'), ('scan', '/scan')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_scan',
            'transform_tolerance': 0.01,
            'min_height': -0.75,
            'max_height': 0.05,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.017453292519943295,
            'scan_time': 0.1,
            'range_min': 0.12,
            'range_max': 12.0,
            'use_inf': True,
        }],
        output='screen',
        condition=IfCondition(lidar_is_3d),
    )

    # ---------- 2c) 启动断言：/scan 只能有一个发布者（仅两种雷达模式各一条） ----------
    # 以前这里只有一行注释「3d 模式绝不桥 gz 自带 /scan」，没有可执行的检查。
    # 双发布时 slam_toolbox 用哪一路取决于到达顺序 → 地图重影/时好时坏，现象完全不指向
    # "多了一路 /scan"，排查代价极高。这条断言把它变成启动时就能看见的红字。
    #   --watch 常驻复查：也能抓到后来才起的野发布者
    #   启动宽限期内 "0 个发布者" 不下结论（桥接与投影是并行起来的）
    scan_guard_3d = ExecuteProcess(
        cmd=[ROS_PYTHON, os.path.join(HERE, 'tools', 'scan_pub_guard.py'),
             '--expect', 'pointcloud_to_laserscan', '--watch'],
        output='screen',
        condition=IfCondition(lidar_is_3d),
    )
    scan_guard_2d = ExecuteProcess(
        cmd=[ROS_PYTHON, os.path.join(HERE, 'tools', 'scan_pub_guard.py'),
             '--expect', 'gz_bridge', '--watch'],
        output='screen',
        condition=UnlessCondition(lidar_is_3d),
    )
    # ---------- 3) odom -> base_footprint TF ----------
    # 两种输入源，用 ekf:=true/false 切换：
    #   默认     /odom                ← gz 真值里程计（A 阶段：只考验 SLAM）
    #   ekf:=true /odometry/filtered  ← EKF 融合（C 阶段：轮式 + IMU，贴近真实机器人）
    odom_tf = ExecuteProcess(
        cmd=[ROS_PYTHON, ODOM_TF, '--ros-args', '-p', 'odom_topic:=/odom'],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('ekf'), "' != 'true'"])),
    )
    odom_tf_ekf = ExecuteProcess(
        cmd=[ROS_PYTHON, ODOM_TF, '--ros-args', '-p',
             'odom_topic:=/odometry/filtered'],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('ekf'), "' == 'true'"])),
    )
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        parameters=[EKF_PARAMS, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('ekf')),
    )

    # ---------- 4b) 相机 TF：已删除（2026-10-23） ----------
    # 原先是补一条 base_link -> camera_link 静态 TF，因为官方 URDF 里那个帧的位置是旧的。
    # 现在 URDF 换成 Lighthouse 自己那份（models/lighthouse/robot.urdf），camera_link 就在
    # (-0.004, 0, 0.690)，robot_state_publisher 会自己发布 —— 再发静态 TF 就是两个发布者
    # 抢同一个帧（TF 会报 TF_OLD_DATA，tf2_echo 会来回打印两个不同的值）。

    # ---------- 4) robot_state_publisher（Lighthouse URDF -> TF 链） ----------
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     # mesh_dir 传绝对路径，URDF 里用 $(arg mesh_dir)/meshes/... 引用网格，
                     # 这样既不依赖 turtlebot3_description 的安装位置，也不用改 xacro 的 find。
                     # lidar:=2d/3d 两位分别指向 models/lighthouse 与 models/lighthouse_3d。
                     'robot_description': Command(['xacro ', lh_urdf, ' mesh_dir:=', lh_model])}],
        output='screen',
    )

    # ---------- 5) slam_toolbox ----------
    # slam:=true 或 explore:=true 都要建图（探索需要实时地图作为规划依据）
    slam_or_explore = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('slam'), "' == 'true' or '",
         LaunchConfiguration('explore'), "' == 'true'"]))
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': SLAM_PARAMS,
                          'use_sim_time': use_sim_time}.items(),
        condition=slam_or_explore,
    )

    # ---------- 5b) 前沿探索所需的"实况导航" ----------
    # 注意：这里用 navigation_launch（不含 map_server / amcl），
    # 因为探索时地图来自 slam_toolbox 的实时 /map，而不是磁盘上的旧图。
    nav_live = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('explore_params'),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    frontier = ExecuteProcess(
        cmd=[ROS_PYTHON, FRONTIER_NODE,
             '--max-goals', LaunchConfiguration('explore_max_goals'),
             '--max-minutes', LaunchConfiguration('explore_max_minutes'),
             '--min-cluster', LaunchConfiguration('explore_min_cluster'),
             '--revisit-gap', LaunchConfiguration('explore_revisit_gap'),
             '--goal-clearance', LaunchConfiguration('explore_goal_clearance'),
             '--min-goal-dist', LaunchConfiguration('explore_min_goal_dist'),
             '--stall-secs', LaunchConfiguration('explore_stall_secs'),
             '--map-prefix', LaunchConfiguration('explore_map_prefix'),
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    # ---------- 6) Nav2 ----------
    # 定位方式二选一（loc 参数）：
    #   amcl  —— 用 AMCL 做 map->odom（真实机器人必须这样）
    #   truth —— 静态 map->odom 恒等，直接信 /odom（仿真里 /odom 就是真值）
    #
    # 为什么要 truth：仿真里 /odom 就是真值，让 AMCL 去"修正"只会引入额外误差源；
    # 语义导航阶段先用 truth 把"感知-语言-导航"跑通，AMCL 调参留作独立课题。
    #
    # 更正（2026-10-26）：早先记录过"AMCL 在这张图上跳 map->odom 1.25 m + 124°、
    # 把 TF 链带偏 3 m"，Dan 确认那次是他人为设置仿真造成的，不是系统缺陷 ——
    # 结论（仿真用 truth）不变，但不要再照着这个"故障"去查 AMCL。
    # 两个条件都用 PythonExpression 单表达式算出来。
    # 注意 AndSubstitution 只接受 (left, right) 两个位置参数，传列表会报
    # TypeError: AndSubstitution.__init__() missing 1 required positional argument: 'right'
    nav_truth = PythonExpression([
        "('", LaunchConfiguration('nav'), "' == 'true') and ('",
        LaunchConfiguration('loc'), "' == 'truth')",
    ])
    nav_amcl = PythonExpression([
        "('", LaunchConfiguration('nav'), "' == 'true') and ('",
        LaunchConfiguration('loc'), "' != 'truth')",
    ])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('nav2_params'),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            # 下面两个必须显式写 "False"（大写 F）：
            #   nav2_bringup 内部会执行 PythonExpression(['not ', slam])，
            #   如果我们把自己的 slam:=false 漏进去，就变成 eval("not false")
            #   -> NameError: name 'false' is not defined
            'slam': 'False',
            # 拆成独立进程，方便 ros2 node list / 单独重启某个 server
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(nav_amcl),
    )

    # ---------- 6b) loc:=truth —— 不起 AMCL，改用静态 map->odom ----------
    map_server_truth = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': use_sim_time}],
        condition=IfCondition(nav_truth),
    )
    lifecycle_truth = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'],
                     'use_sim_time': use_sim_time}],
        condition=IfCondition(nav_truth),
    )
    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom_truth', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(nav_truth),
    )
    nav_truth_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params'),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(nav_truth),
    )

    # ---------- 7) RViz ----------
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', RVIZ_CFG],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ---------- 8) 自动巡游建图 ----------
    # tour:=true 时自动把整栋房子走一遍
    # 速度/转速可以在命令行透传：tour_speed:=0.15 tour_spin_rate:=0.3
    tour = ExecuteProcess(
        cmd=[ROS_PYTHON, TOUR_NODE,
             '--speed', LaunchConfiguration('tour_speed'),
             '--spin-rate', LaunchConfiguration('tour_spin_rate'),
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('tour')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='2d',
                              description='雷达版本：2d = 单线（models/lighthouse，默认，'
                                          '行为与以前完全一致）；3d = 64 线（models/lighthouse_3d，'
                                          '点云 + pointcloud_to_laserscan 投影出 /scan）。'
                                          '3d 需要先跑 python3 build_world.py --lidar 3d'),
        DeclareLaunchArgument('slam', default_value='false', description='启动 slam_toolbox 建图'),
        DeclareLaunchArgument('nav', default_value='false', description='启动 Nav2 导航'),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP, description='导航地图 yaml'),
        DeclareLaunchArgument('nav2_params', default_value=NAV2_PARAMS_NAV,
                              description='导航模式(Nav2)参数文件；建图/探索固定用 nav2_params.yaml'),
        DeclareLaunchArgument('loc', default_value='amcl',
                              description='定位方式：amcl = AMCL 做 map->odom（真实）；'
                                          'truth = 静态 map->odom，直接信 /odom（仿真真值，'
                                          '仿真里的推荐做法 —— /odom 即真值，不必让 AMCL 修正）'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('nvidia', default_value='false',
                              description='true = 用 PRIME 把 Gazebo 渲染丢给独显（双显卡笔记本提速）'),
        DeclareLaunchArgument('ekf', default_value='false',
                              description='true = 用 EKF 融合轮式里程计 + IMU 作为 odom '
                                          '（C 阶段，贴近真实机器人）；'
                                          'false = 用 gz 真值里程计（A 阶段）'),
        DeclareLaunchArgument('tour', default_value='false',
                              description='true = 自动巡游建图（配合 slam:=true 用）'),
        DeclareLaunchArgument('tour_speed', default_value='0.20',
                              description='巡游最大线速度 m/s（越小建图越干净，0.12~0.20）'),
        DeclareLaunchArgument('tour_spin_rate', default_value='0.50',
                              description='巡游原地旋转角速度 rad/s（越小建图越干净，0.25~0.50）'),
        DeclareLaunchArgument('explore', default_value='false',
                              description='true = 前沿探索（自己把可达区域探完，与 tour 互斥使用）'),
        DeclareLaunchArgument('explore_max_goals', default_value='120',
                              description='探索最多发多少个导航目标（40 个只到 61.8%，整屋要 100+）'),
        DeclareLaunchArgument('explore_max_minutes', default_value='60.0',
                              description='探索时间上限（仿真分钟，超时自动存图退出）'),
        DeclareLaunchArgument('explore_min_cluster', default_value='12',
                              description='小于该格数的 frontier 簇当噪声丢掉'),
        DeclareLaunchArgument('explore_revisit_gap', default_value='0.4',
                              description='失败目标的拉黑半径 m（别超过 0.5，否则小地图会死锁）'),
        DeclareLaunchArgument('explore_goal_clearance', default_value='0.40',
                              description='目标格周围必须有的净空 m（贴墙目标会让 Nav2 卡死）'),
        DeclareLaunchArgument('explore_min_goal_dist', default_value='0.60',
                              description='最终目标离车的最小距离 m（太小会瞬间完成、白烧目标）'),
        DeclareLaunchArgument('explore_stall_secs', default_value='12.0',
                              description='卡死看门狗秒数：目标被接受后这么久内位移不足 0.25 m '
                                          '就取消并拉黑。A/B 实测（2026-10-24）12 s 与 18 s 无收益差别'
                                          '（卡死 2 vs 4 次），保持 12；放宽只会让坏目标多烧时间'),
        # 规划器 A/B 开关：默认 SmacPlanner2D；传 config/nav2_params_explore_navfn.yaml 即为对照组。
        # 两份文件只差 planner 插件，其余（BT、进度判定、膨胀）完全一致，才叫干净对照。
        DeclareLaunchArgument('explore_params', default_value=NAV2_PARAMS_EXPLORE,
                              description='探索用 Nav2 参数文件（默认 SmacPlanner2D；'
                                          '传 config/nav2_params_explore_navfn.yaml 换回 NavFn 做对照）'),
        DeclareLaunchArgument('explore_map_prefix',
                              default_value='dan_house_lighthouse_explore',
                              description='探索存图前缀（检查点+最终图），默认带 lighthouse 以免覆盖旧图'),

        *offload_to_nvidia,
        gz_gui,
        gz_server,
        bridge_2d,
        bridge_3d,
        # 投影节点跟着仿真一起起（它只在有人订阅 /scan 时才处理点云，早起无害）
        TimerAction(period=3.0, actions=[projector]),
        # /scan 单发布者断言：与桥接/投影同时起，自己做启动宽限期。
        # 3d 与 2d 各一条（互斥条件），常驻复查，出现双发布时打红字。
        scan_guard_3d,
        scan_guard_2d,
        ekf,
        odom_tf,
        odom_tf_ekf,
        TimerAction(period=2.0, actions=[robot_state_publisher]),
        TimerAction(period=4.0, actions=[slam]),
        TimerAction(period=6.0, actions=[nav2]),
        # loc:=truth 时的替代方案：map_server + 生命周期 + 静态 map->odom + navigation
        TimerAction(period=4.0, actions=[static_map_odom]),
        TimerAction(period=5.0, actions=[map_server_truth, lifecycle_truth]),
        TimerAction(period=6.0, actions=[nav_truth_2]),
        # 探索模式的实况导航 + 探索节点（等 nav2 的 action server 起来再发目标）
        # 注意：nav2 的 lifecycle_manager 在机器负载高时可能卡在 Configuring，
        # 所以这里把 nav_live 推迟到 20 s，给 bridge/slam/生命周期切换留足时间
        TimerAction(period=20.0, actions=[nav_live]),
        TimerAction(period=30.0, actions=[frontier]),
        # 巡游等 SLAM + TF 都起来再走，别一开机就冲
        TimerAction(period=10.0, actions=[tour]),
        rviz,
    ])
