from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ra6a", package_name="ra6a_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
