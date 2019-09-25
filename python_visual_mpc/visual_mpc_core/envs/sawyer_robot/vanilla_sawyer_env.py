from python_visual_mpc.visual_mpc_core.envs.sawyer_robot.base_sawyer_env import BaseSawyerEnv
import copy


class VanillaSawyerEnv(BaseSawyerEnv):
    def __init__(self, env_params, _=None):
        self._hyper = copy.deepcopy(env_params)
        BaseSawyerEnv.__init__(self, env_params)
        self._adim, self._sdim = self._base_adim, self._base_sdim

    def _next_qpos(self, action):
        assert action.shape[0] == self._base_adim, "Action should have shape (5,)"
        return self._previous_target_qpos * self.mode_rel + action