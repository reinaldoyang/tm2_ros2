from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

GRIPPER_COMPORT = '/dev/ttyUSB1'


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        description='IP address of the TM robot',
    )

    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        name='tm_driver',
        output='screen',
        arguments=[['robot_ip:=', LaunchConfiguration('robot_ip')]],
    )

    gripper_driver_node = Node(
        package='robotiq_85_driver',
        executable='robotiq_85_driver',
        name='robotiq_85_driver',
        output='screen',
        parameters=[{
            'num_grippers': 1,
            'comport': GRIPPER_COMPORT,
            'baud': ParameterValue('115200', value_type=str),
        }],
        remappings=[('/gripper/stat', '/robotiq_gripper/state')],
    )

    return LaunchDescription([
        robot_ip_arg,
        tm_driver_node,
        gripper_driver_node,
    ])
