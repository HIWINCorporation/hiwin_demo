from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hiwin_ra6_moveit_config"),
                    "config",
                    "ra6.urdf.xacro",
                ]
            ),
            " ",
            "name:=",
            "ra605_710",
            " ",
            "ra_type:=",
            "ra605_710",
            " ",
            "use_fake_hardware:=",
            "true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    send_trajectory_node = Node(
        package="hiwin_demo",
        executable="sample",
        output="both",
        parameters=[
            robot_description,
        ],
        arguments=["--controller_name", "/manipulator_controller"],
    )

    nodes_to_start = [send_trajectory_node]

    return LaunchDescription(nodes_to_start)
