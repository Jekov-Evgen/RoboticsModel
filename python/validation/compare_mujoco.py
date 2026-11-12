"""Cross-checks of the Robotics Model engine against closed-form solutions and against MuJoCo.

    pip install mujoco matplotlib
    python python/validation/compare_mujoco.py [--plot docs/images/validation.png] [--json results.json]

For every scenario three things are compared:
  * REF   - an analytic solution, or MuJoCo with RK4 and a tiny time step (the "ground truth" curve);
  * ours  - Robotics Model at its default 240 Hz step (4 substeps);
  * MuJoCo(240 Hz) - MuJoCo's default Euler integrator at the same 1/240 s step, so both engines get the same budget.

Controllers are PD loops evaluated at 240 Hz with the torque held between evaluations (what the engine does), so the
comparison measures integration, constraint solving and contact handling rather than control-loop differences.
Everything is deterministic; the script prints a table and exits non-zero if a tolerance is exceeded.
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import mujoco
import numpy as np

import robotics_model as rm

G = 9.81
DT = 1.0 / 240.0


# --------------------------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------------------------


def mj_model(body_xml: str, dt: float, integrator: str = "Euler", gravity=(0, -G, 0), extra: str = "") -> mujoco.MjModel:
    xml = f"""
    <mujoco>
      <option timestep="{dt}" gravity="{gravity[0]} {gravity[1]} {gravity[2]}" integrator="{integrator}"/>
      <worldbody>
        {body_xml}
      </worldbody>
      {extra}
    </mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def mj_run(model, data, seconds, record, control=None, control_period=DT):
    """Steps MuJoCo for `seconds`, calling control() every control_period (torque held in between)."""
    n = int(round(seconds / model.opt.timestep))
    hold = max(1, int(round(control_period / model.opt.timestep)))
    out = []
    for k in range(n):
        if control is not None and k % hold == 0:
            control(model, data)
        mujoco.mj_step(model, data)
        out.append(record(model, data))
    return np.array(out)


def resample(t_src, y_src, t_dst):
    y_src = np.asarray(y_src)
    if y_src.ndim == 1:
        return np.interp(t_dst, t_src, y_src)
    return np.stack([np.interp(t_dst, t_src, y_src[:, i]) for i in range(y_src.shape[1])], axis=1)


def rk4(f, y0, t_end, dt):
    ts = np.arange(0, t_end + dt / 2, dt)
    ys = np.zeros((len(ts), len(y0)))
    y = np.array(y0, dtype=float)
    ys[0] = y
    for i in range(1, len(ts)):
        k1 = f(y)
        k2 = f(y + dt / 2 * k1)
        k3 = f(y + dt / 2 * k2)
        k4 = f(y + dt * k3)
        y = y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        ys[i] = y
    return ts, ys


class Result:
    def __init__(self, name, description, tolerance_deg=None):
        self.name = name
        self.description = description
        self.curves = {}  # label -> (t, y)
        self.metrics = {}
        self.tolerance = tolerance_deg
        self.ylabel = "angle, deg"

    def add(self, label, t, y):
        self.curves[label] = (np.asarray(t), np.asarray(y))


# --------------------------------------------------------------------------------------------------------
# scenario 1: free pendulum released from the horizontal (analytic reference)
# --------------------------------------------------------------------------------------------------------


def pendulum_ours():
    w = rm.World()
    base = w.add_body("box", size=(0.2, 0.2, 0.2), mass=0, position=(0, 2, 0))
    link = w.add_body("box", size=(1.0, 0.1, 0.1), mass=2, position=(0.5, 2, 0))
    j = w.connect(base, link, "hinge", anchor=(0, 2, 0), axis=(0, 0, 1))
    j.set_controller("speed_motor")
    j.max_effort = 0  # motor off: free swing
    ts, ys = [], []
    for _ in range(int(3.0 / DT)):
        w.step()
        ts.append(w.time)
        ys.append(math.degrees(j.angle))
    return np.array(ts), np.array(ys)


def substep_study():
    """Error of the free pendulum against the analytic solution for 1, 4 (default), 8 and 16 substeps."""
    m, L, th = 2.0, 1.0, 0.1
    inertia = m * (L**2 + th**2) / 12 + m * (L / 2) ** 2
    ts, ys = rk4(lambda y: np.array([y[1], -m * G * (L / 2) * math.cos(y[0]) / inertia]), [0, 0], 3.0, 1e-4)
    ref = np.degrees(ys[:, 0])
    out = {}
    for substeps in (1, 4, 8, 16):
        w = rm.World()
        w.substeps = substeps
        base = w.add_body("box", size=(0.2, 0.2, 0.2), mass=0, position=(0, 2, 0))
        link = w.add_body("box", size=(1.0, 0.1, 0.1), mass=2, position=(0.5, 2, 0))
        j = w.connect(base, link, "hinge", anchor=(0, 2, 0), axis=(0, 0, 1))
        j.set_controller("speed_motor")
        j.max_effort = 0
        t, y = [], []
        for _ in range(int(3.0 / DT)):
            w.step()
            t.append(w.time)
            y.append(math.degrees(j.angle))
        out[substeps] = float(np.max(np.abs(np.array(y) - np.interp(t, ts, ref))))
    return out


def scenario_pendulum():
    r = Result("pendulum", "free pendulum released from horizontal, 3 s", tolerance_deg=2.5)
    t_o, y_o = pendulum_ours()
    # analytic: J theta'' = -m g (L/2) cos(theta)
    m, L, th = 2.0, 1.0, 0.1
    inertia = m * (L**2 + th**2) / 12 + m * (L / 2) ** 2
    ts, ys = rk4(lambda y: np.array([y[1], -m * G * (L / 2) * math.cos(y[0]) / inertia]), [0, 0], 3.0, 1e-4)
    ref = np.degrees(ys[:, 0])
    r.add("analytic", ts, ref)
    r.add("ours (240 Hz)", t_o, y_o)
    body = '<body pos="0.5 2 0"><joint type="hinge" axis="0 0 1" pos="-0.5 0 0"/>' \
           '<geom type="box" size="0.5 0.05 0.05" mass="2" contype="0" conaffinity="0"/></body>'
    model = mj_model(body, DT)
    data = mujoco.MjData(model)
    y = mj_run(model, data, 3.0, lambda m_, d: math.degrees(d.qpos[0]))
    r.add("MuJoCo (240 Hz)", (np.arange(len(y)) + 1) * DT, y)
    r.reference = "analytic"
    return r


# --------------------------------------------------------------------------------------------------------
# scenario 2: PD-controlled arm holding against gravity, then a step (MuJoCo RK4 as reference)
# --------------------------------------------------------------------------------------------------------


def scenario_pd_step():
    r = Result("pd_step", "PD joint under gravity: hold, then a 30 deg step at 0.5 s, 4 s", tolerance_deg=0.7)
    kp, kd, target = 100.0, 20.0, math.radians(30)

    w = rm.World()
    base = w.add_body("box", size=(0.2, 0.2, 0.2), mass=0, position=(0, 2, 0))
    link = w.add_body("box", size=(1.0, 0.1, 0.1), mass=2, position=(0.5, 2, 0))
    j = w.connect(base, link, "hinge", anchor=(0, 2, 0), axis=(0, 0, 1))
    j.max_effort = 1000
    j.set_controller("pd", kp=kp, kd=kd, derivative_filter=0.0005)
    j.set_step(0.0, target, start_time=0.5)
    ts, ys = [], []
    for _ in range(int(4.0 / DT)):
        w.step()
        ts.append(w.time)
        ys.append(math.degrees(j.angle))
    r.add("ours (240 Hz)", ts, ys)

    body = '<body pos="0.5 2 0"><joint name="j" type="hinge" axis="0 0 1" pos="-0.5 0 0"/>' \
           '<geom type="box" size="0.5 0.05 0.05" mass="2" contype="0" conaffinity="0"/></body>'
    extra = '<actuator><motor joint="j" gear="1"/></actuator>'

    def run(integrator, dt):
        model = mj_model(body, dt, integrator, extra=extra)
        data = mujoco.MjData(model)

        def control(m_, d):
            ref = target if d.time >= 0.5 else 0.0
            d.ctrl[0] = kp * (ref - d.qpos[0]) - kd * d.qvel[0]

        y = mj_run(model, data, 4.0, lambda m_, d: math.degrees(d.qpos[0]), control)
        return (np.arange(len(y)) + 1) * dt, y

    t_ref, y_ref = run("RK4", 1.0 / 2400)
    r.add("MuJoCo RK4 (reference)", t_ref, y_ref)
    t_m, y_m = run("Euler", 1.0 / 960)  # same 4 substeps per 240 Hz controller tick as ours
    r.add("MuJoCo (240 Hz control)", t_m, y_m)
    r.reference = "MuJoCo RK4 (reference)"
    return r


# --------------------------------------------------------------------------------------------------------
# scenario 3: double pendulum (coupled nonlinear dynamics), reference = MuJoCo RK4 with a tiny step
# --------------------------------------------------------------------------------------------------------


def scenario_double_pendulum():
    r = Result("double_pendulum", "free double pendulum from horizontal, 1.5 s (chaotic: compare the early part)", tolerance_deg=2.0)
    w = rm.World()
    base = w.add_body("box", size=(0.2, 0.2, 0.2), mass=0, position=(0, 2, 0))
    a = w.add_body("box", size=(1.0, 0.1, 0.1), mass=2, position=(0.5, 2, 0))
    b = w.add_body("box", size=(0.8, 0.1, 0.1), mass=1, position=(1.4, 2, 0))
    j1 = w.connect(base, a, "hinge", anchor=(0, 2, 0), axis=(0, 0, 1))
    j2 = w.connect(a, b, "hinge", anchor=(1.0, 2, 0), axis=(0, 0, 1))
    for j in (j1, j2):
        j.set_controller("speed_motor")
        j.max_effort = 0
    ts, y1, y2 = [], [], []
    for _ in range(int(1.5 / DT)):
        w.step()
        ts.append(w.time)
        y1.append(math.degrees(j1.angle))
        y2.append(math.degrees(j2.angle))
    r.add("ours q1", ts, y1)
    r.add("ours q2", ts, y2)
    body = ('<body pos="0.5 2 0"><joint name="j1" type="hinge" axis="0 0 1" pos="-0.5 0 0"/>'
            '<geom type="box" size="0.5 0.05 0.05" mass="2" contype="0" conaffinity="0"/>'
            '<body pos="0.9 0 0"><joint name="j2" type="hinge" axis="0 0 1" pos="-0.4 0 0"/>'
            '<geom type="box" size="0.4 0.05 0.05" mass="1" contype="0" conaffinity="0"/></body></body>')

    def run(integrator, dt):
        model = mj_model(body, dt, integrator)
        data = mujoco.MjData(model)
        y = mj_run(model, data, 1.5, lambda m_, d: [math.degrees(d.qpos[0]), math.degrees(d.qpos[1])])
        return (np.arange(len(y)) + 1) * dt, y

    t_ref, y_ref = run("RK4", 1.0 / 4800)
    r.add("MuJoCo RK4 q1", t_ref, y_ref[:, 0])
    r.add("MuJoCo RK4 q2", t_ref, y_ref[:, 1])
    t_m, y_m = run("Euler", DT)
    r.add("MuJoCo (240 Hz) q1", t_m, y_m[:, 0])
    r.add("MuJoCo (240 Hz) q2", t_m, y_m[:, 1])
    r.pairs = [("ours q1", "MuJoCo RK4 q1"), ("ours q2", "MuJoCo RK4 q2"),
               ("MuJoCo (240 Hz) q1", "MuJoCo RK4 q1"), ("MuJoCo (240 Hz) q2", "MuJoCo RK4 q2")]
    return r


# --------------------------------------------------------------------------------------------------------
# scenario 4: the 3-DOF manipulator of the course project, PD joints, steps on all joints
# --------------------------------------------------------------------------------------------------------

ARM_INERTIA = {"yaw": 4.5308, "shoulder": 4.49854, "elbow": 0.574313}  # kg m^2 about each axis, home pose
ARM_WN = {"yaw": 8.0, "shoulder": 8.0, "elbow": 16.0}


def arm_gains():
    out = {}
    for k, J in ARM_INERTIA.items():
        wn = ARM_WN[k]
        out[k] = (J * wn * wn, 2.0 * wn * J)  # zeta = 1
    return out


def arm_ours():
    w = rm.World()
    base = w.add_body("box", size=(0.7, 0.2, 0.7), mass=0, position=(0, 0.1, 0))
    turret = w.add_body("box", size=(0.22, 0.4, 0.22), mass=4, position=(0, 0.4, 0))
    j1 = w.connect(base, turret, "hinge", anchor=(0, 0.2, 0), axis=(0, 1, 0))
    upper = w.add_body("box", size=(0.8, 0.13, 0.13), mass=3, position=(0.4, 0.6, 0))
    j2 = w.connect(turret, upper, "hinge", anchor=(0, 0.6, 0), axis=(0, 0, 1))
    fore = w.add_body("box", size=(0.7, 0.1, 0.1), mass=2, position=(1.15, 0.6, 0))
    j3 = w.connect(upper, fore, "hinge", anchor=(0.8, 0.6, 0), axis=(0, 0, 1))
    tool = w.add_body("sphere", size=(0.14, 0.14, 0.14), mass=0.5, position=(1.5, 0.6, 0))
    w.connect(fore, tool, "fixed", anchor=(1.5, 0.6, 0))
    gains = arm_gains()
    for j, name, amp, start in ((j1, "yaw", 50, 1.0), (j2, "shoulder", 35, 1.5), (j3, "elbow", 55, 2.0)):
        kp, kd = gains[name]
        j.max_effort = 1000
        j.set_controller("pd", kp=kp, kd=kd, derivative_filter=0.0005)
        j.set_step(0.0, math.radians(amp), start_time=start)
    return w, (j1, j2, j3), tool


ARM_XML = """
<body name="base" pos="0 0.1 0"><geom type="box" size="0.35 0.1 0.35" contype="0" conaffinity="0"/>
 <body name="turret" pos="0 0.3 0"><joint name="yaw" type="hinge" axis="0 1 0" pos="0 -0.2 0"/>
  <geom type="box" size="0.11 0.2 0.11" mass="4" contype="0" conaffinity="0"/>
  <body name="upper" pos="0.4 0.2 0"><joint name="shoulder" type="hinge" axis="0 0 1" pos="-0.4 0 0"/>
   <geom type="box" size="0.4 0.065 0.065" mass="3" contype="0" conaffinity="0"/>
   <body name="fore" pos="0.75 0 0"><joint name="elbow" type="hinge" axis="0 0 1" pos="-0.35 0 0"/>
    <geom type="box" size="0.35 0.05 0.05" mass="2" contype="0" conaffinity="0"/>
    <geom name="tool" type="sphere" size="0.07" mass="0.5" pos="0.35 0 0" contype="0" conaffinity="0"/>
   </body></body></body></body>"""


def scenario_manipulator():
    r = Result("manipulator", "3-DOF manipulator, PD on every joint, steps of 50/35/55 deg at 1/1.5/2 s, 5 s", tolerance_deg=1.0)
    w, joints, tool = arm_ours()
    ts, ys, tool_xyz = [], [], []
    for _ in range(int(5.0 / DT)):
        w.step()
        ts.append(w.time)
        ys.append([math.degrees(j.angle) for j in joints])
        tool_xyz.append(np.array(tool.position))
    ys = np.array(ys)
    tool_xyz = np.array(tool_xyz)
    extra = ('<actuator><motor joint="yaw" gear="1"/><motor joint="shoulder" gear="1"/><motor joint="elbow" gear="1"/></actuator>')
    gains = arm_gains()
    steps = [(math.radians(50), 1.0), (math.radians(35), 1.5), (math.radians(55), 2.0)]
    names = ["yaw", "shoulder", "elbow"]

    def run(integrator, dt):
        model = mj_model(ARM_XML, dt, integrator, extra=extra)
        data = mujoco.MjData(model)
        tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tool")

        def control(m_, d):
            for i, name in enumerate(names):
                kp, kd = gains[name]
                ref = steps[i][0] if d.time >= steps[i][1] else 0.0
                d.ctrl[i] = kp * (ref - d.qpos[i]) - kd * d.qvel[i]

        def record(m_, d):
            return np.concatenate([np.degrees(d.qpos[:3]), d.geom_xpos[tool_id]])

        y = mj_run(model, data, 5.0, record, control)
        return (np.arange(len(y)) + 1) * dt, y

    t_ref, y_ref = run("RK4", 1.0 / 2400)
    t_m, y_m = run("Euler", 1.0 / 960)
    for i, name in enumerate(names):
        r.add(f"ours {name}", ts, ys[:, i])
        r.add(f"MuJoCo RK4 {name}", t_ref, y_ref[:, i])
        r.add(f"MuJoCo (240 Hz) {name}", t_m, y_m[:, i])
    r.pairs = [(f"ours {n}", f"MuJoCo RK4 {n}") for n in names] + [(f"MuJoCo (240 Hz) {n}", f"MuJoCo RK4 {n}") for n in names]
    r.tool = (np.array(ts), tool_xyz, t_ref, y_ref[:, 3:6], t_m, y_m[:, 3:6])
    return r


# --------------------------------------------------------------------------------------------------------
# scenario 5: sliding box with Coulomb friction (analytic reference)
# --------------------------------------------------------------------------------------------------------


def scenario_friction():
    r = Result("friction", "box sliding on a slope (tilted gravity), mu = 0.4, 1.5 s", tolerance_deg=None)
    r.ylabel = "distance slid, m"
    theta, mu = math.radians(30), 0.4
    g = (G * math.sin(theta), -G * math.cos(theta), 0.0)
    w = rm.World()
    w.gravity = g
    box = w.add_body("box", size=(0.4, 0.2, 0.4), mass=1, position=(0, 0.1, 0), friction=mu)
    x0 = box.position[0]
    ts, xs = [], []
    for _ in range(int(1.5 / DT)):
        w.step()
        ts.append(w.time)
        xs.append(box.position[0] - x0)
    r.add("ours (240 Hz)", ts, xs)
    a = G * (math.sin(theta) - mu * math.cos(theta))
    t = np.array(ts)
    r.add("analytic", t, 0.5 * a * t**2)
    body = '<body pos="0 0.1 0"><freejoint/><geom type="box" size="0.2 0.1 0.2" mass="1" friction="0.4 0 0"/></body>'
    extra = ""
    xml_body = body + '<geom type="plane" size="20 20 0.1" euler="-90 0 0" friction="0.4 0 0"/>'
    model = mj_model(xml_body, DT, gravity=g, extra=extra)
    data = mujoco.MjData(model)
    y = mj_run(model, data, 1.5, lambda m_, d: d.qpos[0])
    r.add("MuJoCo (240 Hz)", (np.arange(len(y)) + 1) * DT, y - 0.0)
    r.reference = "analytic"
    r.metric_unit = "mm"
    return r


# --------------------------------------------------------------------------------------------------------
# scenario 6: torque-free asymmetric body (intermediate-axis tumbling): energy and angular momentum
# --------------------------------------------------------------------------------------------------------


def quat_to_matrix(q):
    w_, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w_), 2 * (x * z + y * w_)],
                     [2 * (x * y + z * w_), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w_)],
                     [2 * (x * z - y * w_), 2 * (y * z + x * w_), 1 - 2 * (x * x + y * y)]])


def scenario_tumbling():
    r = Result("tumbling", "torque-free box (0.2 x 0.4 x 0.6) spun about its intermediate axis, 10 s", tolerance_deg=None)
    r.ylabel = "body-frame angular velocity ω_y, rad/s"
    dims = (0.2, 0.4, 0.6)
    m = 1.0
    inertia = np.array([m * (dims[1] ** 2 + dims[2] ** 2) / 12, m * (dims[0] ** 2 + dims[2] ** 2) / 12, m * (dims[0] ** 2 + dims[1] ** 2) / 12])
    omega0 = np.array([0.05, 6.0, 0.05])  # I_y is the middle principal moment: spinning about it is unstable (the body flips)

    def energy_and_L(q, w_world):
        R = quat_to_matrix(q)
        L = R @ np.diag(inertia) @ R.T @ w_world
        return 0.5 * w_world @ L, np.linalg.norm(L)

    def euler_rhs(wb):  # Euler's equations, I w' = (I w) x w
        return np.linalg.solve(np.diag(inertia), np.cross(inertia * wb, wb))

    t_ref, y_ref = rk4(euler_rhs, omega0, 10.0, 1e-4)
    r.add("analytic (Euler eqs.)", t_ref, y_ref[:, 1])

    w = rm.World()
    w.gravity = (0, 0, 0)
    b = w.add_body("box", size=dims, mass=m, position=(0, 5, 0))
    b.angular_velocity = omega0
    e0, l0 = energy_and_L(b.orientation, b.angular_velocity)
    ts, wx, es, ls = [], [], [], []
    for _ in range(int(10.0 / DT)):
        w.step()
        R = quat_to_matrix(b.orientation)
        e, l = energy_and_L(b.orientation, b.angular_velocity)
        ts.append(w.time)
        wx.append((R.T @ b.angular_velocity)[1])
        es.append((e / e0 - 1) * 100)
        ls.append((l / l0 - 1) * 100)
    r.add("ours (240 Hz)", ts, wx)

    model = mj_model(f'<body pos="0 5 0"><freejoint/><geom type="box" size="{dims[0]/2} {dims[1]/2} {dims[2]/2}" mass="{m}" contype="0" conaffinity="0"/></body>', DT, gravity=(0, 0, 0))
    data = mujoco.MjData(model)
    data.qvel[3:6] = omega0  # free-joint angular velocity is in the body frame (identity orientation at t = 0)
    mujoco.mj_forward(model, data)
    iner = model.body_inertia[1].copy()

    def record(m_, d):
        R = d.xmat[1].reshape(3, 3)
        w_world = R @ d.qvel[3:6]
        L = R @ np.diag(iner) @ R.T @ w_world
        return [d.qvel[4], 0.5 * w_world @ L, np.linalg.norm(L)]

    start = record(model, data)
    y = np.array(mj_run(model, data, 10.0, record))
    r.add("MuJoCo (240 Hz)", (np.arange(len(y)) + 1) * DT, y[:, 0])
    r.reference = "analytic (Euler eqs.)"
    r.energy = {"ours": np.array(es), "mujoco": (y[:, 1] / start[1] - 1) * 100}
    r.momentum = {"ours": np.array(ls), "mujoco": (y[:, 2] / start[2] - 1) * 100}
    return r


# --------------------------------------------------------------------------------------------------------
# metrics, report, plot
# --------------------------------------------------------------------------------------------------------


def max_err(r, a, b, t_end=None):
    ta, ya = r.curves[a]
    tb, yb = r.curves[b]
    t = np.arange(DT, min(ta[-1], tb[-1]) + 1e-9, DT)
    if t_end:
        t = t[t <= t_end]
    d = resample(ta, ya, t) - resample(tb, yb, t)
    return float(np.max(np.abs(d))), float(np.sqrt(np.mean(d**2)))


def evaluate(results):
    rows = []
    for r in results:
        if hasattr(r, "pairs"):
            pairs = r.pairs
        else:
            ref = r.reference
            pairs = [(k, ref) for k in r.curves if k != ref]
        for a, b in pairs:
            end = 0.8 if r.name == "double_pendulum" else None
            mx, rms = max_err(r, a, b, end)
            scale = 1000.0 if getattr(r, "metric_unit", "") == "mm" else 1.0
            unit = getattr(r, "metric_unit", "rad/s" if r.name == "tumbling" else "deg")
            rows.append((r.name, a, b, mx * scale, rms * scale, unit))
    return rows


def plot(results, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"ours": "#2a78d6", "mujoco": "#eb6834", "ref": "#1a1a1a"}
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    for ax, r in zip(axes.flat, results):
        for label, (t, y) in r.curves.items():
            if r.name == "manipulator" and not label.endswith("elbow"):
                continue
            if r.name == "double_pendulum" and not label.endswith("q2"):
                continue
            if label.startswith("ours"):
                ax.plot(t, y, color=colors["ours"], lw=2, label=label.replace(" elbow", "").replace(" q2", ""))
            elif "240 Hz" in label and label.startswith("MuJoCo"):
                ax.plot(t, y, color=colors["mujoco"], lw=1.4, ls="--", label=label.replace(" elbow", "").replace(" q2", ""))
            else:
                ax.plot(t, y, color=colors["ref"], lw=1, ls=":", label=label.replace(" elbow", "").replace(" q2", ""))
        title = {"manipulator": "3-DOF manipulator: elbow joint", "double_pendulum": "double pendulum: second joint"}.get(r.name, r.name.replace("_", " "))
        ax.set_title(title, fontsize=10, loc="left", fontweight="bold")
        ax.set_xlabel("time, s")
        ax.set_ylabel(r.ylabel)
        ax.legend(frameon=False, fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main(argv=None):  # noqa: C901
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", help="write a comparison figure (needs matplotlib)")
    ap.add_argument("--json", help="write the numbers to a JSON file")
    args = ap.parse_args(argv)

    results = [scenario_pendulum(), scenario_pd_step(), scenario_double_pendulum(), scenario_manipulator(), scenario_friction(), scenario_tumbling()]
    rows = evaluate(results)

    print(f"{'scenario':<17}{'curve':<26}{'vs':<26}{'max err':>10}{'rms':>10}  unit")
    for name, a, b, mx, rms, unit in rows:
        print(f"{name:<17}{a:<26}{b:<26}{mx:>10.4g}{rms:>10.4g}  {unit}")

    study = substep_study()
    print("\nfree pendulum, max error vs analytic by substeps: " + ", ".join(f"{k}: {v:.2f} deg" for k, v in study.items()))

    tumbling = results[-1]
    e_o, e_m = tumbling.energy["ours"], tumbling.energy["mujoco"]
    l_o, l_m = tumbling.momentum["ours"], tumbling.momentum["mujoco"]
    print(f"\ntumbling box, 10 s: max energy drift ours {np.max(np.abs(e_o)):.2e} %, MuJoCo Euler {np.max(np.abs(e_m)):.2e} %")
    print(f"                    max |L| drift  ours {np.max(np.abs(l_o)):.2e} %, MuJoCo Euler {np.max(np.abs(l_m)):.2e} %")

    manip = results[3]
    t_o, tool_o, t_r, tool_r, t_m, tool_m = manip.tool
    n = min(len(t_o), len(t_r))
    ref_i = resample(t_r, tool_r, t_o)
    mu_i = resample(t_m, tool_m, t_o)
    err_o = np.linalg.norm(tool_o - ref_i, axis=1)
    err_m = np.linalg.norm(mu_i - ref_i, axis=1)
    print(f"manipulator tool position error vs RK4 reference: ours max {err_o.max() * 1000:.1f} mm, MuJoCo(240 Hz) max {err_m.max() * 1000:.1f} mm")

    if args.plot:
        plot(results, args.plot)
        print("figure:", args.plot)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"rows": rows, "tumbling": {"ours_max_energy_drift_pct": float(np.max(np.abs(e_o))), "mujoco_max_energy_drift_pct": float(np.max(np.abs(e_m))),
                                    "ours_max_L_drift_pct": float(np.max(np.abs(l_o))), "mujoco_max_L_drift_pct": float(np.max(np.abs(l_m)))},
                       "pendulum_error_by_substeps_deg": study, "manipulator_tool_error_mm": {"ours": float(err_o.max() * 1000), "mujoco_240": float(err_m.max() * 1000)}}, f, indent=1)

    failures = []
    for r in results:
        if r.tolerance is None:
            continue
        for name, a, b, mx, rms, unit in rows:
            if name == r.name and a.startswith("ours") and mx > r.tolerance:
                failures.append(f"{name}: {a} vs {b}: {mx:.3f} > {r.tolerance}")
    if failures:
        print("\nOUT OF TOLERANCE:\n  " + "\n  ".join(failures))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
