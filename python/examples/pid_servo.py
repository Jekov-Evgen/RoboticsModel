"""A PID servo from scratch: build the scene, tune the controller, log the step response.

    python python/examples/pid_servo.py
"""

import math

import robotics_model as rm

world = rm.World()
base = world.add_body(size=(0.3, 1.0, 0.3), mass=0, position=(0, 0.5, 0))
arm = world.add_body(size=(1.2, 0.16, 0.16), position=(0.6, 1.0, 0))
servo = world.connect(base, arm, "hinge", anchor=(0, 1, 0), axis=(0, 1, 0))
servo.max_effort = 8
servo.set_controller("pid", kp=14, ki=3, kd=4)
servo.set_step(base=0.0, amplitude=1.0, start_time=0.5)  # 1 rad step at t = 0.5 s

print(f"{'t, s':>6} {'target':>8} {'angle':>8} {'torque':>8}")
for k in range(0, 6 * 240):
    world.step()
    if k % 60 == 0:
        print(f"{world.time:6.2f} {servo.reference(world.time):8.3f} {servo.angle:8.3f} {servo.effort:8.3f}")

print(f"\nfinal angle {math.degrees(servo.angle):.2f} deg, constraint drift {world.max_anchor_error():.1e} m")
