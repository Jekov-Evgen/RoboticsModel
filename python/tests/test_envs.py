import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")

import robotics_model.envs as envs  # noqa: E402  (registers the ids)
from robotics_model.envs import arm  # noqa: E402


def test_forward_kinematics_matches_the_simulated_tool():
    """The analytic FK used for goals must agree with the engine's tool position."""
    world = arm.make_arm_world()
    q = np.radians([35.0, 25.0, 60.0])
    world.set_joint_targets(q)
    world.run(6.0)
    sim = np.array(world.body("tool").position)
    assert np.allclose(world.joint_angles(), q, atol=2e-3)
    assert np.linalg.norm(sim - arm.tool_position(world.joint_angles())) < 2e-3


def test_inverse_kinematics_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = np.array([rng.uniform(-1.5, 1.5), rng.uniform(0.0, 1.0), rng.uniform(0.3, 1.7)])
        goal = arm.tool_position(q)
        sol = arm.inverse_kinematics(goal)
        assert sol is not None
        assert np.linalg.norm(arm.tool_position(sol) - goal) < 1e-9
    assert arm.inverse_kinematics([3.0, 1.0, 0.0]) is None  # out of reach


def test_environment_passes_the_gymnasium_checker():
    from gymnasium.utils.env_checker import check_env

    check_env(gym.make("RoboticsModel/ArmReach-v0").unwrapped, skip_render_check=True)


def test_registered_env_truncates_at_200_steps():
    env = gym.make("RoboticsModel/ArmReach-v0")
    env.reset(seed=0)
    truncated = False
    for i in range(200):
        _, _, terminated, truncated, _ = env.step(env.action_space.sample())
        assert not terminated
    assert truncated


def test_same_seed_same_episode():
    def episode():
        env = gym.make("RoboticsModel/ArmReach-v0")
        obs, _ = env.reset(seed=7)
        total = 0.0
        rng = np.random.default_rng(3)
        for _ in range(60):
            obs, r, *_ = env.step(rng.uniform(-1, 1, 3))
            total += r
        return obs, total

    (o1, t1), (o2, t2) = episode(), episode()
    assert np.array_equal(o1, o2) and t1 == t2


def oracle_policy(env, obs):
    """Closed-form IK to the goal, then proportional steps on the joint targets."""
    goal = obs[9:12].astype(float)
    q_goal = arm.inverse_kinematics(goal)
    return np.clip((q_goal - env.unwrapped.targets) / env.unwrapped.action_scale, -1, 1)


def test_oracle_reaches_goals_and_random_policy_does_not():
    env = gym.make("RoboticsModel/ArmReach-v0")
    rng = np.random.default_rng(0)
    oracle_success, oracle_dist, random_dist = 0, [], []
    for seed in range(10):
        obs, info = env.reset(seed=seed)
        for _ in range(120):
            obs, _, _, _, info = env.step(oracle_policy(env, obs))
        oracle_success += info["is_success"]
        oracle_dist.append(info["distance"])
        obs, info = env.reset(seed=seed)
        for _ in range(120):
            obs, _, _, _, info = env.step(rng.uniform(-1, 1, 3))
        random_dist.append(info["distance"])
    assert oracle_success >= 9
    assert np.mean(oracle_dist) < 0.5 * np.mean(random_dist)


def test_bad_actions_are_rejected():
    env = gym.make("RoboticsModel/ArmReach-v0").unwrapped
    env.reset(seed=0)
    with pytest.raises(ValueError):
        env.step(np.zeros(4))
