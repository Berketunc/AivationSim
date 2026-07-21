"""Warehouse-avoidance Isaac Lab task, first version (see MILESTONE2_STATUS.md's
research pivot section for the full research context).

The drone is a velocity-commanded rigid body (see _apply_action), not a
thrust/torque-modeled quadrotor — this mirrors path_follower_node.py's
MAVSDK VelocityBodyYawspeed control interface on the real vehicle, and is a
deliberate simplification for this first version. Observations and
collision/out-of-bounds checks are computed analytically from the known,
authored warehouse geometry (see warehouse_avoidance_env_cfg.py) rather than
via a simulated LiDAR/occupancy grid, matching oa_planning's own grid bounds
and goal exactly so the scenario stays comparable to the classical baseline.

The policy only controls (vx, vy) — z is held at a fixed cruise altitude by
a small internal correction, not learned (see warehouse_avoidance_env_cfg.py's
CRUISE_ALTITUDE_Z comment for why: this warehouse's pillars are floor-to-
ceiling, so z movement gives no obstacle-avoidance benefit here). "Goal
reached" accordingly measures successful navigation to the goal region, not
a landing — the actual physical landing is a separate, already-solved
classical behavior out of scope for this policy.

This is a bare, plain-reward RL task meant to prove the training loop works
end to end. The residual-on-classical-controller architecture, IL
pretraining, domain randomization, and the final 5-metric reward shaping are
deliberately not part of this version.
"""

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.markers import CUBOID_MARKER_CFG, VisualizationMarkers

from .warehouse_avoidance_env_cfg import (
    CRUISE_ALTITUDE_Z,
    DRONE_COLLISION_RADIUS,
    GOAL_POS,
    GOAL_REACHED_RADIUS,
    GOAL_XY,
    GRID_ORIGIN,
    GRID_SIZE,
    K_NEAREST_PILLARS,
    OUT_OF_BOUNDS_MARGIN,
    PILLAR_SIZE,
    PILLAR_XY,
    PILLAR_Z,
    WALLS,
    WarehouseAvoidanceEnvCfg,
)


class WarehouseAvoidanceEnv(DirectRLEnv):
    cfg: WarehouseAvoidanceEnvCfg

    def __init__(self, cfg: WarehouseAvoidanceEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._commanded_vel_xy = torch.zeros(self.num_envs, 2, device=self.device)
        self._goal_reached = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._collided = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._out_of_bounds = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        self._goal_xy = torch.tensor(GOAL_XY, device=self.device, dtype=torch.float32)
        self._goal_pos = torch.tensor(GOAL_POS, device=self.device, dtype=torch.float32)  # debug-vis marker only

        # Static geometry, as flat (xy) tensors — pillars/walls both span the
        # full flight-altitude band (0..3.5m), so an xy-only box distance is
        # sufficient for collision/observation purposes; z never needs to
        # factor in.
        self._pillar_xy = torch.tensor(PILLAR_XY, device=self.device, dtype=torch.float32)
        self._pillar_half_xy = torch.tensor(
            [PILLAR_SIZE[0] / 2, PILLAR_SIZE[1] / 2], device=self.device, dtype=torch.float32
        )
        self._wall_xy = torch.tensor([w[0][:2] for w in WALLS], device=self.device, dtype=torch.float32)
        self._wall_half_xy = torch.tensor(
            [[s[0] / 2, s[1] / 2] for _, s in WALLS], device=self.device, dtype=torch.float32
        )

        self._episode_sums = {
            key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            for key in ["distance_to_goal", "time_penalty", "collision", "goal_reached"]
        }

        self.set_debug_vis(self.cfg.debug_vis)

    def _setup_scene(self):
        self._robot = RigidObject(self.cfg.robot)
        self.scene.rigid_objects["robot"] = self._robot

        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        # Static obstacles: spawn once under env_0 with literal (non-regex)
        # prim paths, then clone_environments() below replicates the whole
        # authored env_0 subtree (walls, pillars, and the robot spawned
        # above via its regex prim path) into every other env. No
        # rigid_props (no RigidBodyAPI) — collision_props alone is the
        # cheapest static-collider form, appropriate for fixed geometry.
        env0 = self.scene.env_prim_paths[0]
        for i, (pos, size) in enumerate(WALLS):
            wall_cfg = sim_utils.CuboidCfg(size=size, collision_props=sim_utils.CollisionPropertiesCfg())
            wall_cfg.func(f"{env0}/wall_{i}", wall_cfg, translation=pos)
        for i, (x, y) in enumerate(PILLAR_XY):
            pillar_cfg = sim_utils.CuboidCfg(size=PILLAR_SIZE, collision_props=sim_utils.CollisionPropertiesCfg())
            pillar_cfg.func(f"{env0}/pillar_{i}", pillar_cfg, translation=(x, y, PILLAR_Z))

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor):
        self._commanded_vel_xy = actions.clone().clamp(-1.0, 1.0) * self.cfg.max_speed_mps

    def _apply_action(self):
        # Re-issued every physics substep (DirectRLEnv.step()'s decimation
        # loop) so the body tracks the commanded velocity the same way
        # MAVSDK's VelocityBodyYawspeed inner loop does on the real vehicle.
        # z is not policy-controlled (see CRUISE_ALTITUDE_Z's docstring in
        # the cfg) — held here by a small proportional correction instead.
        pos_z = self._robot.data.root_pos_w[:, 2] - self._terrain.env_origins[:, 2]
        vz = torch.clamp(
            self.cfg.altitude_hold_kp * (CRUISE_ALTITUDE_Z - pos_z),
            -self.cfg.altitude_hold_max_speed_ms,
            self.cfg.altitude_hold_max_speed_ms,
        )
        lin_vel = torch.cat([self._commanded_vel_xy, vz.unsqueeze(-1)], dim=-1)
        angular = torch.zeros_like(lin_vel)
        self._robot.write_root_velocity_to_sim(torch.cat([lin_vel, angular], dim=-1))

    def _local_pos(self) -> torch.Tensor:
        """Root position in env-local frame (env origin subtracted)."""
        return self._robot.data.root_pos_w - self._terrain.env_origins

    def _obstacle_clearance(self, pos_xy: torch.Tensor) -> torch.Tensor:
        """Signed distance from pos_xy (N,2) to the nearest pillar or wall
        surface (min over all obstacles), via the standard box-SDF clamp
        trick. Negative means inside an obstacle."""
        pillar_delta = (pos_xy.unsqueeze(1) - self._pillar_xy.unsqueeze(0)).abs() - self._pillar_half_xy
        pillar_outside = pillar_delta.clamp(min=0.0).norm(dim=-1)
        pillar_inside = pillar_delta.max(dim=-1).values.clamp(max=0.0)
        pillar_dist = (pillar_outside + pillar_inside).min(dim=-1).values

        wall_delta = (pos_xy.unsqueeze(1) - self._wall_xy.unsqueeze(0)).abs() - self._wall_half_xy
        wall_outside = wall_delta.clamp(min=0.0).norm(dim=-1)
        wall_inside = wall_delta.max(dim=-1).values.clamp(max=0.0)
        wall_dist = (wall_outside + wall_inside).min(dim=-1).values

        return torch.minimum(pillar_dist, wall_dist)

    def _get_observations(self) -> dict:
        pos_local = self._local_pos()
        goal_rel_xy = self._goal_xy - pos_local[:, :2]
        lin_vel_xy = self._robot.data.root_lin_vel_w[:, :2]

        drone_xy = pos_local[:, :2].unsqueeze(1)
        rel_xy = self._pillar_xy.unsqueeze(0) - drone_xy
        dist = torch.norm(rel_xy, dim=-1)
        _, idx = torch.topk(dist, K_NEAREST_PILLARS, largest=False, dim=-1)
        nearest_rel = torch.gather(rel_xy, 1, idx.unsqueeze(-1).expand(-1, -1, 2))
        nearest_rel_flat = nearest_rel.reshape(self.num_envs, -1)

        x_min, y_min, _ = GRID_ORIGIN
        sx, sy, _ = GRID_SIZE
        clearance = torch.stack(
            [
                pos_local[:, 0] - x_min,
                (x_min + sx) - pos_local[:, 0],
                pos_local[:, 1] - y_min,
                (y_min + sy) - pos_local[:, 1],
            ],
            dim=-1,
        )

        obs = torch.cat([goal_rel_xy, lin_vel_xy, nearest_rel_flat, clearance], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        pos_local = self._local_pos()
        dist_to_goal = torch.linalg.norm(self._goal_xy - pos_local[:, :2], dim=-1)
        dist_mapped = 1 - torch.tanh(dist_to_goal / 3.0)

        rewards = {
            "distance_to_goal": dist_mapped * self.cfg.distance_to_goal_reward_scale * self.step_dt,
            "time_penalty": torch.full((self.num_envs,), self.cfg.time_penalty_scale, device=self.device)
            * self.step_dt,
            "collision": self._collided.float() * self.cfg.collision_penalty,
            "goal_reached": self._goal_reached.float() * self.cfg.goal_reached_bonus,
        }
        reward = torch.sum(torch.stack(list(rewards.values())), dim=0)
        for key, value in rewards.items():
            self._episode_sums[key] += value
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos_local = self._local_pos()

        # 2D — "goal reached" means successfully navigated to the goal
        # region, not landed (see cfg's CRUISE_ALTITUDE_Z comment).
        dist_to_goal = torch.linalg.norm(self._goal_xy - pos_local[:, :2], dim=-1)
        self._goal_reached = dist_to_goal < GOAL_REACHED_RADIUS

        clearance = self._obstacle_clearance(pos_local[:, :2])
        self._collided = clearance < DRONE_COLLISION_RADIUS

        # z is actively held at CRUISE_ALTITUDE_Z (_apply_action), so this
        # is a safety net against a runaway altitude-hold correction, not
        # a bound the policy needs to learn to respect — x/y are the only
        # axes it actually controls.
        x_min, y_min, z_min = GRID_ORIGIN
        sx, sy, sz = GRID_SIZE
        m = OUT_OF_BOUNDS_MARGIN
        self._out_of_bounds = (
            (pos_local[:, 0] < x_min - m)
            | (pos_local[:, 0] > x_min + sx + m)
            | (pos_local[:, 1] < y_min - m)
            | (pos_local[:, 1] > y_min + sy + m)
            | (pos_local[:, 2] < z_min - m)
            | (pos_local[:, 2] > z_min + sz + m)
        )

        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = self._goal_reached | self._collided | self._out_of_bounds
        return terminated, time_out

    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES

        pos_local_xy = (self._robot.data.root_pos_w[env_ids] - self._terrain.env_origins[env_ids])[:, :2]
        final_distance_to_goal = torch.linalg.norm(self._goal_xy - pos_local_xy, dim=1).mean()
        extras = dict()
        for key in self._episode_sums.keys():
            episodic_sum_avg = torch.mean(self._episode_sums[key][env_ids])
            extras["Episode_Reward/" + key] = episodic_sum_avg / self.max_episode_length_s
            self._episode_sums[key][env_ids] = 0.0
        self.extras["log"] = dict()
        self.extras["log"].update(extras)
        extras = dict()
        extras["Episode_Termination/goal_reached"] = torch.count_nonzero(self._goal_reached[env_ids]).item()
        extras["Episode_Termination/collision"] = torch.count_nonzero(self._collided[env_ids]).item()
        extras["Episode_Termination/out_of_bounds"] = torch.count_nonzero(self._out_of_bounds[env_ids]).item()
        extras["Episode_Termination/time_out"] = torch.count_nonzero(self.reset_time_outs[env_ids]).item()
        extras["Metrics/final_distance_to_goal"] = final_distance_to_goal.item()
        self.extras["log"].update(extras)

        self._robot.reset(env_ids)
        super()._reset_idx(env_ids)
        if len(env_ids) == self.num_envs:
            # Spread out resets to avoid training spikes when many envs reset together.
            self.episode_length_buf = torch.randint_like(self.episode_length_buf, high=int(self.max_episode_length))

        self._commanded_vel_xy[env_ids] = 0.0
        self._goal_reached[env_ids] = False
        self._collided[env_ids] = False
        self._out_of_bounds[env_ids] = False

        default_root_state = self._robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self._terrain.env_origins[env_ids]
        self._robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_pos_visualizer"):
                marker_cfg = CUBOID_MARKER_CFG.copy()
                marker_cfg.markers["cuboid"].size = (0.2, 0.2, 0.2)
                marker_cfg.prim_path = "/Visuals/Command/goal_position"
                self.goal_pos_visualizer = VisualizationMarkers(marker_cfg)
            self.goal_pos_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pos_visualizer"):
                self.goal_pos_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        goal_pos_w = self._goal_pos.unsqueeze(0) + self._terrain.env_origins
        self.goal_pos_visualizer.visualize(goal_pos_w)
