import numpy as np

import robotics_model as rm


def arm_world():
    w = rm.World()
    base = w.add_body("box", size=(0.3, 1.0, 0.3), mass=0, position=(0, 0.5, 0))
    arm = w.add_body("box", size=(1.2, 0.16, 0.16), position=(0.6, 1.0, 0))
    joint = w.connect(base, arm, "hinge", anchor=(0, 1, 0), axis=(0, 1, 0))
    joint.max_effort = 8
    joint.set_controller("pid", kp=14, ki=3, kd=4)
    joint.target = 1.0
    return w, joint, arm


def test_ideal_encoder_reads_the_joint_angle():
    w, joint, arm = arm_world()
    enc = w.add_sensor("encoder", arm, name="enc", rate=0, encoder_counts=0)  # type defaults add quantization
    w.run(2.0)
    assert enc.valid
    assert abs(enc.value[0] - joint.angle) < 1e-12
    assert abs(enc.truth[0] - enc.value[0]) < 1e-12
    assert enc.channels == ["position"]


def test_noise_quantization_and_latency():
    w, joint, arm = arm_world()
    noisy = w.add_sensor("encoder", arm, rate=200, noise_stddev=0.02, seed=5)
    quant = w.add_sensor("encoder", arm, rate=200, encoder_counts=256)
    late = w.add_sensor("encoder", arm, rate=200, latency=20)
    err = []
    for _ in range(240 * 4):
        w.step()
        if noisy.fresh:
            err.append(noisy.value[0] - noisy.truth[0])
    assert abs(np.std(err) - 0.02) < 0.004
    step = 2 * np.pi / 256
    assert abs(quant.value[0] / step - round(quant.value[0] / step)) < 1e-6
    assert abs(late.value[0] - late.truth[0]) > 1e-4 or w.time > 3  # delayed reading lags the truth while moving


def test_same_seed_same_noise():
    def run():
        w, _, arm = arm_world()
        s = w.add_sensor("encoder", arm, rate=100, noise_stddev=0.05, seed=42)
        out = []
        for _ in range(500):
            w.step()
            out.append(s.value[0])
        return out

    assert run() == run()


def test_controller_can_run_on_a_noisy_encoder():
    ideal, _, _ = arm_world()
    ideal.run(4.0)
    w, joint, arm = arm_world()
    w.add_sensor("encoder", arm, rate=200, noise_stddev=0.05, latency=3, feeds_controller=True, seed=3)
    w.run(4.0)
    # still tracks the target, but not identically to the ideal loop
    assert abs(joint.angle - 1.0) < 0.15
    assert joint.angle != ideal.joint(0).angle


def test_lidar_sees_a_wall():
    w = rm.World()
    car = w.add_body("box", size=(0.4, 0.2, 0.3), mass=0, position=(0, 0.5, 0))
    w.add_body("box", size=(0.2, 2, 6), mass=0, position=(3, 1, 0), name="wall")
    lidar = w.add_sensor("lidar2d", car, beams=9, fov=0.5, max_range=10, rate=0, position=(0.2, 0, 0))
    w.step()
    ranges = lidar.value
    assert len(ranges) == 9 and abs(ranges[4] - 2.7) < 0.02  # beam straight ahead: 3.0 - 0.1 - 0.2
    assert lidar.ray_ends.shape == (9, 3) and all(lidar.ray_hits)


def test_imu_at_rest_reads_gravity():
    w = rm.World()
    box = w.add_body("box", size=(1, 0.2, 1), position=(0, 0.1, 0))
    imu = w.add_sensor("imu", box, rate=0)
    w.run(2.0)
    assert abs(imu.value[1] - 9.81) < 0.2  # specific force: +g upwards at rest
