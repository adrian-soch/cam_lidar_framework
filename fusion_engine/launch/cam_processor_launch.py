from launch import LaunchDescription
from launch.actions import ExecuteProcess

ABS_PATH_TO_FUSION_ENGINE = '/home/adrian/dev/ros2_ws/src/cam_lidar_tools/fusion_engine/fusion_engine'


def generate_launch_description():
    # Run the camera processing script in the correct folder
    #   becuase there are many includes and colcon build doesnt like
    #   running it as a fomrmal package
    execute_camera_processor = ExecuteProcess(
        cmd=[[
            'python3 ./camera_processing_node.py'
        ]],
        cwd=[ABS_PATH_TO_FUSION_ENGINE],
        shell=True,
        name='camera_processor',
        emulate_tty=True
    )

    return LaunchDescription([
        execute_camera_processor,
    ])
