# oa_rl — Isaac Lab warehouse-avoidance RL task

Isaac Lab task for the classical-vs-RL obstacle-avoidance research
described in `../MILESTONE2_STATUS.md`'s research pivot section. Lives
outside `~/IsaacLab` (kept as a generic, gitignored external engine —
same pattern as `~/PX4-Autopilot` vs `sim_assets/`, `~/open_vins` vs
`oa_vio/`), following Isaac Lab's own "external project" convention so it
registers with Isaac Lab's existing training scripts.

**First version only** (see the `WarehouseAvoidanceEnv` docstring): a bare
Isaac Lab reproduction of `sim_assets/worlds/warehouse.sdf`'s room/pillar
layout with a plain RL reward, to prove the training loop runs end to end.
The residual-on-classical-controller architecture, IL pretraining, domain
randomization, and the final 5-metric reward shaping are deliberately not
part of this version.

## Setup

```bash
source ~/lab/bin/activate
python -m pip install -e source/oa_rl
```

## Verify (before any real training run)

```bash
source ~/lab/bin/activate
timeout 120 python scripts/random_agent.py \
  --task Isaac-WarehouseAvoidance-Direct-v0 --num_envs 4 --headless
```

Confirms package registration, scene spawn (floor + walls + 14 pillars ×
N envs + drone), and reset/step cycles all work before committing to a
real training run.

## Train

```bash
source ~/lab/bin/activate
python scripts/rsl_rl/train.py \
  --task Isaac-WarehouseAvoidance-Direct-v0 --headless --num_envs 64
```
