# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Talos robot."""

from __future__ import annotations

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

##
# Configuration
##

TALOS_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Talos",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/holmes/simulations/isaac/IsaacLab/source/extensions/omni.isaac.lab_assets/data/Robots/PalRobotics/Talos/talos_fake_grippers.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=None,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.11),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "talos": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness={
                "*": 2.0,
            },
            damping={
                ".*": 5.0,
            },
        ),
    },
)
"""Configuration for the Talos robot."""
