""" This file defines the base class for the policy. """
import abc, six
from funcsigs import signature, Parameter
from tensorflow.contrib.training import HParams

def get_policy_args(policy, obs, t, i_tr, step_data=None):
    policy_args = {}
    policy_signature = signature(policy.act)  # Gets arguments required by policy
    for arg in policy_signature.parameters:  # Fills out arguments according to their keyword
        value = policy_signature.parameters[arg].default
        if arg in obs:
            value = obs[arg]
        elif step_data is not None and arg in step_data:
            value = step_data[arg]

        # everthing that is not cached in post_process_obs is assigned here:
        elif arg == 't':
            value = t
        elif arg == 'i_tr':
            value = i_tr
        elif arg == 'obs':           # policy can ask for all arguments from environment
            value = obs
        elif arg == 'step_data':
            value = step_data
        elif arg == 'goal_pos':
            value = step_data['goal_pos']

        if value is Parameter.empty:
            # required parameters MUST be set by agent
            raise ValueError("Required Policy Param {} not set in agent".format(arg))
        policy_args[arg] = value
    # import pdb; pdb.set_trace()
    return policy_args


@six.add_metaclass(abc.ABCMeta)
class Policy(object):

    def override_defaults(self, policyparams):
        if 'custom_sampler' in policyparams:
            for name, value in policyparams['custom_sampler'].get_default_hparams().items():
                if name in self._hp:
                    print('Warning default value for {} already set!'.format(name))
                    self._hp.set_hparam(name, value)
                else:
                    self._hp.add_hparam(name, value)

        for name, value in policyparams.items():
            if name == 'type':
                continue      # type corresponds to policy class

            print('overriding param {} to value {}'.format(name, value))
            if value == getattr(self._hp, name):
                raise ValueError("attribute is {} is identical to default value!!".format(name))
            self._hp.set_hparam(name, value)

    def _default_hparams(self):
        return HParams()

    @abc.abstractmethod
    def act(self, *args):
        """
        Args:
            Request necessary arguments in definition
            (see Agent code)
        Returns:
            A dict of outputs D
               -One key in D, 'actions' should have the action for this time-step
        """
        raise NotImplementedError("Must be implemented in subclass.")

    def reset(self):
        pass


class DummyPolicy(object):
    def __init__(self, ag_params, policyparams, gpu_id, ngpu):
        """ Computes actions from states/observations. """
        pass

    @abc.abstractmethod
    def act(self, *args):
        pass

    def reset(self):
        pass


class NullPolicy(Policy):
    """
    Returns 0 for all timesteps
    """
    def __init__(self,  ag_params, policyparams, gpu_id, ngpu):
        self._adim = ag_params['adim']

    def act(self):
        return np.zeros(self._adim)
