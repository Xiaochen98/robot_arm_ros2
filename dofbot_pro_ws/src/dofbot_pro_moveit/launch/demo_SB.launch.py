from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # 1️⃣ 构建 MoveIt 配置（URDF / SRDF / controllers 等）
    moveit_config = MoveItConfigsBuilder(
        "DOFBOT_Pro-V24",
        package_name="dofbot_pro_moveit"
    ).to_moveit_configs()

    # 2️⃣ 生成官方 demo launch
    #    ⚠️ 注意：这里内部“默认会注入 OMPL”
    ld = generate_demo_launch(moveit_config)

    # 3️⃣ 我们手动覆盖 move_group 的参数
    #    强制：只使用 Pilz，不允许 OMPL 出现
    pilz_only_override = {
        "move_group": {
            "ros__parameters": {
                # 默认规划器 → Pilz
                "default_planning_pipeline": "pilz_industrial_motion_planner",

                # 只允许一个 pipeline（不包含 ompl）
                "planning_pipelines": [
                    "pilz_industrial_motion_planner"
                ],

                # Pilz 插件本身的配置
                "pilz_industrial_motion_planner": {
                    "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
                    "default_planner_config": "PTP",
                    "request_adapters": ""
                },
            }
        }
    }

    # 4️⃣ 找到 move_group 这个 Node，把参数塞进去
    for action in ld.entities:
        if hasattr(action, "executable") and action.executable == "move_group":
            if action.parameters is None:
                action.parameters = []
            action.parameters.append(pilz_only_override)
            break

    return ld

