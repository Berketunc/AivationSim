from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

##
# Warehouse geometry — transcribed exactly from sim_assets/worlds/warehouse.sdf
# and precision_landing_ws/src/oa_planning/config/planner_params.yaml. Keep in
# sync if the Gazebo world ever changes.
##

# (pos_xyz, size_xyz). Unlike sim_assets/worlds/warehouse.sdf (which
# deliberately omits the west wall — safe there only because the real
# classical controller always flies +X toward the goal and never explores
# -X), the west wall is kept here: an RL policy explores in every
# direction, and spawn (-8.5, 0) sits only 1.5m from x=-10 — without a
# wall there, early/random exploration drifted out of bounds to the west
# almost immediately, confirmed live (every episode: out_of_bounds=1.0,
# mean episode length ~34 steps / ~1.4s, never reaching row A's pillars at
# x=-5). This is a deliberate, justified divergence from the Gazebo world,
# not an oversight — the two simulators serve different purposes here.
WALLS = (
    ((0.0, 7.0, 1.75), (20.4, 0.2, 3.5)),  # north
    ((0.0, -7.0, 1.75), (20.4, 0.2, 3.5)),  # south
    ((10.0, 0.0, 1.75), (0.2, 14.0, 3.5)),  # east
    ((-10.0, 0.0, 1.75), (0.2, 14.0, 3.5)),  # west
)

PILLAR_SIZE = (0.4, 0.4, 4.0)
PILLAR_Z = 2.0
PILLAR_XY = (
    # Row A (x=-5)
    (-5.0, -6.0), (-5.0, -3.0), (-5.0, 0.0), (-5.0, 3.0), (-5.0, 6.0),
    # Row B (x=0), offset half-pitch from A/C so gaps never line up (the slalom)
    (0.0, -4.5), (0.0, -1.5), (0.0, 1.5), (0.0, 4.5),
    # Row C (x=5), same as row A
    (5.0, -6.0), (5.0, -3.0), (5.0, 0.0), (5.0, 3.0), (5.0, 6.0),
)

# This warehouse's pillars are floor-to-ceiling (4.0m tall, taller than the
# 3.5m walls — see README.md's Milestone 2 section: "there's no altitude at
# which the maze can be flown over"). So z-axis movement provides zero
# obstacle-avoidance benefit for this task — it's a pure 2D navigation
# problem at a fixed cruise altitude. z is therefore held fixed by a small
# internal correction (see env's _apply_action), not policy-controlled;
# action_space/observation_space below only carry x/y. This also means
# "goal reached" here measures successful navigation to the goal region,
# not a landing — the actual physical landing (descent + ArUco alignment)
# is a separate, already-solved classical behavior (path_follower_node's
# SEARCH_MARKER/ALIGN_MARKER/DESCEND_MARKER sequence), deliberately out of
# scope for the RL policy.
SPAWN_XY = (-8.5, 0.0)
GOAL_XY = (8.5, 0.0)
CRUISE_ALTITUDE_Z = 1.5  # matches control_params.yaml's takeoff_altitude_m
GOAL_POS = (*GOAL_XY, CRUISE_ALTITUDE_Z)  # for the (optional) debug-vis marker only
GOAL_REACHED_RADIUS = 0.3

# x500 quad is roughly 0.7m rotor-tip to rotor-tip; treat as a point with
# this clearance radius for the analytic collision check (see env's
# _get_dones — no contact sensors/LiDAR simulated for this first version).
DRONE_COLLISION_RADIUS = 0.35

# Same bounding box oa_planning's grid uses (planner_params.yaml origin/size).
GRID_ORIGIN = (-10.0, -7.0, 0.0)
GRID_SIZE = (20.0, 14.0, 3.5)
OUT_OF_BOUNDS_MARGIN = 0.5

K_NEAREST_PILLARS = 6


@configclass
class WarehouseAvoidanceEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 45.0
    decimation = 4
    action_space = 2  # (vx, vy) — see the CRUISE_ALTITUDE_Z comment above for why z is excluded
    observation_space = 2 + 2 + 2 * K_NEAREST_PILLARS + 4  # goal_rel_xy, lin_vel_xy, nearest pillars, wall clearance
    state_space = 0
    # Off by default: the goal-marker VisualizationMarkers instancer is a
    # likely cause of a real hang seen during headless verification (Kit
    # logged "FabricManager::initializePointInstancer mismatched prototypes
    # on point instancer: /Visuals/Command/goal_position" immediately before
    # the process stopped responding for over an hour, ignoring SIGTERM).
    # Not needed for headless training/verification anyway — only useful
    # when watching a run in the GUI.
    debug_vis = False

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 100,
        render_interval=decimation,
    )
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        debug_vis=False,
    )

    # scene — env_spacing must be >= the room footprint (20.4m) to avoid
    # visual overlap between cloned envs; inter-env physics isolation is
    # separately guaranteed by filter_collisions, independent of spacing.
    # num_envs=64 is a starting default for the first correctness pass, not
    # a final training count.
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=64, env_spacing=25.0, replicate_physics=True)

    # robot: a simple box standing in for the x500 quad, velocity-commanded
    # directly every physics substep (see WarehouseAvoidanceEnv._apply_action)
    # rather than modeled via thrust/torque — mirrors path_follower_node.py's
    # MAVSDK VelocityBodyYawspeed interface, not a full flight-dynamics model.
    robot: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.CuboidCfg(
            size=(0.3, 0.3, 0.12),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=2.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.2, 0.1)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(*SPAWN_XY, CRUISE_ALTITUDE_Z)),
    )

    # Deliberately faster than the real controller's conservative
    # cruise_speed_ms=0.4 (control_params.yaml) — this v1 task is a bare RL
    # warm-up to prove the training loop works, not yet compared against the
    # classical baseline, so matching that number isn't required here.
    max_speed_mps = 1.5

    # Internal (non-learned) altitude hold — see the CRUISE_ALTITUDE_Z
    # comment above. Fast enough to correct quickly since this isn't
    # something the policy should ever need to fight.
    altitude_hold_kp = 2.0
    altitude_hold_max_speed_ms = 1.0

    # reward scales
    distance_to_goal_reward_scale = 15.0
    time_penalty_scale = -0.5
    collision_penalty = -20.0
    goal_reached_bonus = 20.0
