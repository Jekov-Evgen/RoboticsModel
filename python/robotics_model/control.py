"""Helpers that drive a World along planned motions."""

from __future__ import annotations

import numpy as np

from ._core import Chain, Trajectory, World


def follow_trajectory(world: World, chain: Chain, trajectory: Trajectory, settle: float = 0.5):
    """Commands the chain's joint controllers along `trajectory` and steps the world.

    Each physics step the setpoint of every actuated joint of the chain is set to the trajectory position at the
    current time (the joints need a position controller: ``joint.set_controller("pid", ...)``). After the end the
    final pose is held for `settle` seconds. Returns a dict with the arrays ``time``, ``command`` and ``actual``
    (shape (N, dofs)) and ``tip`` (N, 3), recorded after every physics step.

    The joints are PID loops without gravity or velocity feedforward, so the executed motion lags and sags relative
    to the plan; the error grows with the speed of the trajectory (see docs/PLANNING.md). Plan with lower limits
    when path accuracy matters.
    """
    from ._core import DEFAULT_TIME_STEP

    dt = DEFAULT_TIME_STEP
    joints = [world.joint(i) for i in chain.actuated]
    n_steps = int(round((trajectory.duration + settle) / dt))
    t0 = world.time
    times, commands, actual, tips = [], [], [], []
    for _ in range(n_steps):
        q, _, _ = trajectory.sample(world.time - t0)
        for joint, value in zip(joints, q):
            joint.target = float(value)
        world.step(1, dt)
        times.append(world.time - t0)
        commands.append(q)
        actual.append([j.angle for j in joints])
        tips.append(np.array(chain.tip.position))
    return {"time": np.array(times), "command": np.array(commands), "actual": np.array(actual), "tip": np.array(tips)}
