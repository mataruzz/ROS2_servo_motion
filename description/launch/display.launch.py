from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Default paths
    servo_description_path = get_package_share_path('description')
    default_model_path = servo_description_path / 'urdf/servo.xacro'
    default_rviz_config_path = servo_description_path / 'config/config.rviz'



    # Default ros arguments
    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                        description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                        description='Absolute path to rviz config file')
    use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='false', choices=['true', 'false'],
                                        description='Flag to launch rviz')
    
    

    # URDF/xacro reading  
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                   value_type=str)
       
    # Nodes definitions
    robot_state_publisher_node = Node(  
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='log'
    )

        # Condition in which, based on gui parameter, will be launched normal joint_state_publisher node, or ..._gui node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        use_rviz,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])