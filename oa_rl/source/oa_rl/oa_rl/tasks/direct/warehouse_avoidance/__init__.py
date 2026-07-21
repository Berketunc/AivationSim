import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-WarehouseAvoidance-Direct-v0",
    entry_point=f"{__name__}.warehouse_avoidance_env:WarehouseAvoidanceEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.warehouse_avoidance_env_cfg:WarehouseAvoidanceEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WarehouseAvoidancePPORunnerCfg",
    },
)
