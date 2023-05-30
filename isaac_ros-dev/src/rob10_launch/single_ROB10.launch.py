from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    eye = perform_substitutions(context, [LaunchConfiguration('eye')])

    aruco_single_params = {
        #'dct_components_to_remove': LaunchConfiguration('dct_filter_size'),
        #'image_is_rectified': True,
        #'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        #'camera_frame': 'camera_mount',
        #'marker_frame': LaunchConfiguration('marker_frame'),
        #'corner_refinement': LaunchConfiguration('corner_refinement'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        #parameters=[aruco_single_params],
        remappings=[('/camera_info', '/camera_info_left'),
                    ('/image', '/rgb_left')],
    )

    return [aruco_single]


def generate_launch_description():

    dct_filter_size_arg = DeclareLaunchArgument(
        'dct_filter_size', default_value='2',
        description='dct filter size. ',
    )

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='0',
        description='Marker ID. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.34',
        description='Marker size in m. '
    )

    eye_arg = DeclareLaunchArgument(
        'eye', default_value='left',
        description='Eye. ',
        choices=['left', 'right'],
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be refered. '
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='chassis_link',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(dct_filter_size_arg)
    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(eye_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame)
    ld.add_action(corner_refinement_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
