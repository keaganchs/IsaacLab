from __future__ import annotations

from omni.isaac.lab_assets import TALOS_CFG

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.envs import DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.direct.locomotion.locomotion_env import LocomotionEnv


@configclass
class TalosEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 2
    action_scale = 1.0
    num_actions = 30
    num_observations = 102
    num_states = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

    # robot
    robot: ArticulationCfg = TALOS_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    joint_gears: list = [
        # 106.0000, # back_bkz
        # 445.0000, # back_bky
        # 300.0000, # back_bkx

        # 87.0000, # l_arm_shz
        # 99.0000, # l_arm_shx
        # 63.0000, # l_arm_ely
        # 112.0000, # l_arm_elx
        # 25.0000, # l_arm_wry
        # 25.0000, # l_arm_wrx

        # 87.0000, # r_arm_shz
        # 99.0000, # r_arm_shx
        # 63.0000, # r_arm_ely
        # 112.0000, # r_arm_elx
        # 25.0000, # r_arm_wry
        # 25.0000, # r_arm_wrx

        # 404.0000, # hip_flexion_l
        # 404.0000, # hip_adduction_l
        # 210.0000, # hip_rotation_l
        # 648.0000, # knee_l
        # 400.0000, # ankle_l
        # 232.0000, # leg_akx_l

        # 404.0000, # hip_flexion_r
        # 404.0000, # hip_adduction_r
        # 210.0000, # hip_rotation_r
        # 648.0000, # knee_r
        # 400.0000, # ankle_r
        # 232.0000, # leg_akx_r
        
        # Only defined gear ratios that I could find were in talos_robot/talos_description/srdf/talos.srdf. All:=100.
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
        100.0000,
    ]

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.01
    actions_cost_scale: float = 0.01
    alive_reward_scale: float = 2.0
    dof_vel_scale: float = 0.1

    death_cost: float = -1.0
    termination_height: float = 0.8

    angular_velocity_scale: float = 0.25
    contact_force_scale: float = 0.01


class TalosEnv(LocomotionEnv):
    cfg: TalosEnvCfg

    def __init__(self, cfg: TalosEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
