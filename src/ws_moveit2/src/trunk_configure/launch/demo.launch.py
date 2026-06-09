# import os
# from ament_index_python.packages import get_package_share_directory


from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # eyou_bringup_pkg = get_package_share_directory('eyou_ethercat_bringup')
    # ethercat_urdf_file = os.path.join(eyou_bringup_pkg, 'urdf', 'trunk_robot.ethercat.xacro')


    moveit_config = (
        MoveItConfigsBuilder("trunk_robot", package_name="trunk_configure")
        # .robot_description(file_path=ethercat_urdf_file)

        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)