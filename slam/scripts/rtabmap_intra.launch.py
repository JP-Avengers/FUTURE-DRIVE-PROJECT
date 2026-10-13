"""bag 재생 → depth 정렬 → RGB-D 오도메트리 → RTAB-Map 을 한 컨테이너에서 intra-process로 실행.
대용량 이미지가 DDS를 거치지 않아 WSL에서의 프레임 손실이 없다.
인자: bag:=<dir> db:=<file> rate:=0.5 duration:=0(전체) imu:=false rtabmap_args:="--Vis/MaxDepth 6 ..."
imu:=true 이면 /camera/camera/imu 를 Madgwick 필터로 자세(orientation) 계산 후 오도메트리·맵에 중력 제약으로 사용.
bag_aligned:=true 이면 bag에 녹화된 정렬 depth(aligned_depth_to_color)를 그대로 쓰고 depth 정렬 노드를 띄우지 않음(10/9 이후 촬영본). """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

NS = '/camera/camera'
ALIGNED = f'{NS}/aligned_depth_to_color'
QOS = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'play_qos_intra.yaml')


def setup(context):
    a = lambda k: LaunchConfiguration(k).perform(context)
    toks = a('rtabmap_args').split()
    extra = {toks[i].lstrip('-'): toks[i + 1] for i in range(0, len(toks) - 1, 2)}
    intra = [{'use_intra_process_comms': True}]
    sim = {'use_sim_time': True}
    play = {'storage.uri': a('bag'), 'play.rate': float(a('rate')), 'play.clock_publish_frequency': 100.0,
            'play.disable_keyboard_controls': True, 'play.qos_profile_overrides_path': QOS,
            'play.read_ahead_queue_size': 1000}
    if float(a('duration')) > 0:  # playback_duration 파라미터는 무시되므로 종료 시각으로 지정
        import yaml
        meta = yaml.safe_load(open(os.path.join(a('bag'), 'metadata.yaml')))['rosbag2_bagfile_information']
        start = meta['starting_time']['nanoseconds_since_epoch']
        play['play.playback_until_timestamp'] = int(start + float(a('duration')) * 1e9)
    common = {'frame_id': 'camera_link', 'approx_sync': True, 'approx_sync_max_interval': 0.02,
              'topic_queue_size': 30, 'sync_queue_size': 30, 'wait_for_transform': 0.2, **sim}
    use_imu = a('imu').lower() == 'true'
    bag_aligned = a('bag_aligned').lower() == 'true'
    imu_remap = [('imu', '/imu/data')] if use_imu else []
    rgbd_remap = [('rgb/image', f'{NS}/color/image_raw'), ('rgb/camera_info', f'{NS}/color/camera_info'),
                  ('depth/image', f'{ALIGNED}/image_raw')]
    nodes = [
        ComposableNode(package='rosbag2_transport', plugin='rosbag2_transport::Player', name='player',
                       parameters=[play], extra_arguments=intra),
        *([] if bag_aligned else [
        ComposableNode(package='depth_image_proc', plugin='depth_image_proc::RegisterNode', name='register',
                       parameters=[{'queue_size': 30, **sim}], extra_arguments=intra,
                       remappings=[('depth/image_rect', f'{NS}/depth/image_rect_raw'),
                                   ('depth/camera_info', f'{NS}/depth/camera_info'),
                                   ('rgb/camera_info', f'{NS}/color/camera_info'),
                                   ('depth_registered/image_rect', f'{ALIGNED}/image_raw'),
                                   ('depth_registered/camera_info', f'{ALIGNED}/camera_info')])]),
        *([ComposableNode(package='imu_filter_madgwick', plugin='ImuFilterMadgwickRos', name='imu_filter',
                          parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', **sim}],
                          extra_arguments=intra,
                          remappings=[('imu/data_raw', f'{NS}/imu'), ('imu/data', '/imu/data')])] if use_imu else []),
        ComposableNode(package='rtabmap_odom', plugin='rtabmap_odom::RGBDOdometry', name='rgbd_odometry',
                       namespace='rtabmap', parameters=[{**common, 'publish_tf': True, 'wait_imu_to_init': use_imu}, extra],
                       extra_arguments=intra, remappings=rgbd_remap + imu_remap + [('odom', '/odom')]),
        ComposableNode(package='rtabmap_slam', plugin='rtabmap_slam::CoreWrapper', name='rtabmap',
                       namespace='rtabmap', parameters=[{**common, 'subscribe_depth': True,
                                                         'database_path': a('db')}, extra],
                       extra_arguments=intra, remappings=rgbd_remap + imu_remap + [('odom', '/odom')]),
    ]
    return [ComposableNodeContainer(name='slam_container', namespace='', package='rclcpp_components',
                                    executable='component_container_mt', composable_node_descriptions=nodes,
                                    output='screen', parameters=[sim])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag'), DeclareLaunchArgument('db'),
        DeclareLaunchArgument('rate', default_value='0.5'), DeclareLaunchArgument('duration', default_value='0'),
        DeclareLaunchArgument('rtabmap_args', default_value=''), DeclareLaunchArgument('imu', default_value='false'),
        DeclareLaunchArgument('bag_aligned', default_value='false'),
        OpaqueFunction(function=setup)])
