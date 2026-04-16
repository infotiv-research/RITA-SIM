from contextlib import contextmanager
import inspect
import os
import tempfile

import yaml

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.wrap.reacher.mpc import MpcSolverConfig


def get_or_declare_parameter_value(node, name, default_value):
    """Return a ROS parameter value, declaring it first when older nodes omit it."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)
    return node.get_parameter(name).value


@contextmanager
def build_mpc_load_from_robot_config_kwargs(node, *, world_coll_checker=None):
    """Build compatible MPC config kwargs across cuRobo versions.

    Older cuRobo builds do not expose a direct ``horizon`` kwarg on
    ``MpcSolverConfig.load_from_robot_config`` but still support
    ``override_particle_file``. In that case we generate a tiny temporary
    override file so the requested horizon still reaches cuRobo.
    """
    load_from_robot_config_parameters = inspect.signature(
        MpcSolverConfig.load_from_robot_config
    ).parameters

    requested_horizon = int(
        get_or_declare_parameter_value(node, 'mpc_horizon_steps', 30)
    )
    collision_activation_distance = float(
        get_or_declare_parameter_value(node, 'collision_activation_distance', 0.025)
    )

    kwargs = {
        'store_rollouts': True,
        'step_dt': float(get_or_declare_parameter_value(node, 'mpc_step_dt', 0.03)),
    }
    requested_use_cuda_graph = bool(
        get_or_declare_parameter_value(node, 'mpc_use_cuda_graph', False)
    )
    requested_use_cuda_graph_metrics = bool(
        get_or_declare_parameter_value(node, 'mpc_use_cuda_graph_metrics', False)
    )
    # Use PRIMITIVE collision checker for MPC to avoid Warp mesh SDF kernel
    # crashes after MpcSolver.reset() between steps.  All mesh obstacles are
    # already voxelized into cuboids by the obstacle_manager, so OBB checks
    # give equivalent coverage without the unstable Warp GPU state.
    if 'collision_checker_type' in load_from_robot_config_parameters:
        kwargs['collision_checker_type'] = CollisionCheckerType.PRIMITIVE
        kwargs['collision_cache'] = {'obb': 100}
    if world_coll_checker is not None:
        kwargs['world_coll_checker'] = world_coll_checker
    if 'use_cuda_graph' in load_from_robot_config_parameters:
        kwargs['use_cuda_graph'] = requested_use_cuda_graph
    if 'use_cuda_graph_metrics' in load_from_robot_config_parameters:
        kwargs['use_cuda_graph_metrics'] = requested_use_cuda_graph_metrics
    if 'use_cuda_graph_full_step' in load_from_robot_config_parameters:
        kwargs['use_cuda_graph_full_step'] = False

    override_particle_file = None
    if 'horizon' in load_from_robot_config_parameters:
        kwargs['horizon'] = requested_horizon
    elif 'override_particle_file' in load_from_robot_config_parameters:
        fd, override_particle_file = tempfile.mkstemp(
            prefix='curobo_mpc_override_',
            suffix='.yml',
        )
        os.close(fd)
        with open(override_particle_file, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(
                {
                    'model': {
                        'horizon': requested_horizon,
                    },
                    'mppi': {
                        'use_cuda_graph': False,
                    },
                },
                handle,
                sort_keys=False,
            )
        kwargs['override_particle_file'] = override_particle_file
        node.get_logger().info(
            'Installed cuRobo does not expose MPC horizon override directly; '
            f'using override_particle_file to request horizon={requested_horizon} steps.'
        )
    else:
        node.get_logger().warn(
            'Installed cuRobo does not support MPC horizon override through '
            'load_from_robot_config() or override_particle_file; '
            'using the library default horizon.'
        )

    if 'collision_activation_distance' in load_from_robot_config_parameters:
        kwargs['collision_activation_distance'] = collision_activation_distance
    else:
        node.get_logger().warn(
            'Installed cuRobo does not support MPC collision activation distance '
            'override in load_from_robot_config(); using the library default '
            'collision padding.'
        )

    try:
        yield kwargs
    finally:
        if override_particle_file is not None:
            try:
                os.unlink(override_particle_file)
            except OSError as exc:
                node.get_logger().warn(
                    f'Failed to remove temporary MPC override file {override_particle_file}: {exc}'
                )
