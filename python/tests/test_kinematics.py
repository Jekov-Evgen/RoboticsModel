import numpy as np
import pytest

import robotics_model as rm


@pytest.fixture
def arm(examples):
    return rm.load_scene(str(examples / "manipulator_3dof.robot"))


def test_chain_describes_the_manipulator(arm):
    chain = arm.make_chain("tool")
    assert chain.dofs == 3 and chain.root.name == "base" and chain.tip.name == "tool"
    assert len(chain.joints) == 4 and len(chain.actuated) == 3  # the tool is welded with a fixed joint
    assert np.allclose(chain.coordinates(), 0)


def test_forward_kinematics_matches_the_pose_at_load_and_after_motion(arm):
    chain = arm.make_chain("tool")
    position, orientation = chain.forward(np.zeros(3))
    assert np.allclose(position, arm.body("tool").position, atol=1e-12)
    arm.run(8.0)  # the scene's own PID steps move the arm to (50, 35, 55) deg
    q = chain.coordinates()
    assert np.allclose(np.degrees(q), [50, 35, 55], atol=0.3)
    assert np.linalg.norm(chain.forward(q)[0] - arm.body("tool").position) < 1e-3


def test_jacobian_matches_finite_differences(arm):
    chain = arm.make_chain("tool")
    q = np.array([0.4, 0.3, 0.8])
    jac = chain.jacobian(q)
    assert jac.shape == (6, 3)
    h = 1e-6
    for k in range(3):
        dq = np.zeros(3)
        dq[k] = h
        numeric = (chain.forward(q + dq)[0] - chain.forward(q)[0]) / h
        assert np.allclose(jac[:3, k], numeric, atol=1e-5)


def test_ik_reaches_a_target_and_reports_unreachable_ones(arm):
    chain = arm.make_chain("tool")
    truth = np.radians([30, 25, 60])
    target = chain.forward(truth)[0]
    result = chain.solve_ik(target, seed=[0.1, 0.1, 0.1])
    assert result.converged and result.position_error < 1e-4
    assert np.linalg.norm(chain.forward(result.q)[0] - target) < 1e-4
    far = chain.solve_ik([5.0, 1.0, 0.0])
    assert not far.converged and far.position_error > 3
    with pytest.raises(rm.RoboticsError):
        chain.solve_ik([1, 1, 1], seed=[0.0, 0.0])  # one coordinate per actuated joint is required
    with pytest.raises(KeyError):
        arm.make_chain("no_such_body")


def test_trajectory_planning_respects_limits():
    traj = rm.plan_point_to_point([0, 0], [1.5, -1.0], max_velocity=1.0, max_acceleration=[4.0, 6.0])
    assert abs(traj.duration - rm.min_jerk_duration([0, 0], [1.5, -1.0], 1.0, [4.0, 6.0])) < 0.003
    assert np.abs(traj.velocities).max() <= 1.0 * 1.001
    assert np.abs(traj.accelerations[:, 0]).max() <= 4.0 * 1.001
    q, qd, qdd = traj.sample(traj.duration / 2)
    assert np.allclose(q, [0.75, -0.5], atol=1e-3) and qdd.shape == (2,)
    assert np.allclose(traj.positions[0], [0, 0]) and np.allclose(traj.positions[-1], [1.5, -1.0], atol=1e-9)
    assert np.allclose(traj.velocities[0], 0) and np.allclose(traj.velocities[-1], 0, atol=1e-9)
    with pytest.raises(rm.RoboticsError):
        rm.plan_point_to_point([0], [1], max_velocity=0.0, max_acceleration=1.0)


def test_waypoints_stop_at_each_point():
    traj = rm.plan_waypoints([[0.0], [1.0], [0.2], [2.0]], max_velocity=2.0, max_acceleration=10.0)
    assert np.isclose(traj.positions[-1, 0], 2.0)
    stops = np.sum(np.abs(traj.velocities[1:-1, 0]) < 1e-9)
    assert stops == 2


def test_cartesian_line_executed_in_the_engine(arm):
    """Plan a straight line for the tool, run it through the PID joints, and measure what the engine does."""
    chain = arm.make_chain("tool")
    # Start away from the straight-arm singularity (where the elbow would need infinite speed to move the tool inwards).
    ready = rm.plan_point_to_point(chain.coordinates(), np.radians([20, 30, 70]), 1.0, 4.0)
    rm.follow_trajectory(arm, chain, ready, settle=3.0)
    q0 = chain.coordinates()
    start = chain.forward(q0)[0]
    target = chain.forward(np.radians([-25, 50, 50]))[0]
    traj = chain.plan_line(q0, target, max_velocity=1.0, max_acceleration=4.0)
    assert traj is not None and traj.duration < 5
    # The plan itself is straight to within the IK tolerance.
    plan_tip = np.array([chain.forward(q)[0] for q in traj.positions[::25]])
    direction = (target - start) / np.linalg.norm(target - start)
    off = (plan_tip - start) - np.outer((plan_tip - start) @ direction, direction)
    assert np.linalg.norm(off, axis=1).max() < 2e-3

    def run(max_velocity, max_acceleration):
        world = arm.clone()
        plan = chain.plan_line(q0, target, max_velocity, max_acceleration)
        log = rm.follow_trajectory(world, world.make_chain("tool"), plan, settle=2.0)
        off = (log["tip"] - start) - np.outer((log["tip"] - start) @ direction, direction)
        return np.linalg.norm(off, axis=1).max(), np.linalg.norm(log["tip"][-1] - target), plan.duration

    fast_dev, fast_end, fast_time = run(1.0, 4.0)
    slow_dev, slow_end, slow_time = run(0.3, 1.0)
    # PID joints have no velocity or gravity feedforward: path accuracy depends on speed.
    assert fast_dev < 0.08 and fast_end < 0.02
    assert slow_time > 2 * fast_time and slow_dev < 0.5 * fast_dev and slow_end < 0.02
    assert chain.plan_line(q0, [4, 1, 0], 1.0, 4.0) is None
