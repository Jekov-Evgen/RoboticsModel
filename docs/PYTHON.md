# Python API

Привязки к ядру на pybind11: тот же движок, что в редакторе и консоли, но из Python. Вся симуляция работает без окна.

## Установка

```sh
python -m venv .venv && . .venv/bin/activate
pip install .            # из корня репозитория; нужны компилятор C++20 и CMake
pip install -e ".[test]" # для разработки: тесты и editable-режим
python -m pytest         # тесты Python-слоя
```

Зависимости: `numpy` (всегда), `pybind11` и `scikit-build-core` (только при сборке). Сам движок остаётся без сторонних библиотек; привязки — отдельная цель CMake `robotics_py` (`-DROBOTICS_BUILD_PYTHON=ON`).

## За 30 секунд

```python
import robotics_model as rm

world = rm.World()
base = world.add_body(size=(0.3, 1.0, 0.3), mass=0, position=(0, 0.5, 0))
arm = world.add_body(size=(1.2, 0.16, 0.16), position=(0.6, 1.0, 0))
servo = world.connect(base, arm, "hinge", anchor=(0, 1, 0), axis=(0, 1, 0))
servo.max_effort = 8
servo.set_controller("pid", kp=14, ki=3, kd=4)
servo.target = 1.0          # рад

world.run(5.0)              # секунды симуляции, шаг 1/240 с
print(servo.angle)          # 1.0180 — тот же результат, что и в C++
```

Готовые примеры: [`python/examples/pid_servo.py`](../python/examples/pid_servo.py) и [`python/examples/manipulator.py`](../python/examples/manipulator.py).

## Модель объектов

`Body`, `Joint` и `Sensor` — лёгкие **дескрипторы**: они хранят мир и индекс и каждый раз читают актуальные данные из `World`, поэтому остаются корректными, когда мир растёт (`add_body` может переразместить память). Мир живёт, пока жив хоть один дескриптор.

| Класс | Что умеет |
|---|---|
| `World` | `add_body`, `connect`, `add_sensor`, `step(n)`, `run(seconds)`, `clone()`, `copy_state_from()`, `raycast`, `contacts()`, `dumps()/loads()`, массивы `positions()`, `joint_angles()`, `set_joint_targets()` |
| `Body` | `position`, `orientation` (w, x, y, z), `velocity`, `mass`, `friction`, `apply_force()`, `apply_torque()`, `local_to_world()` |
| `Joint` | `angle`, `speed`, `effort`, `set_controller()`, `target`, `set_step()`, `set_sine()`, `set_limits()`, `motor_speed`, `reaction_force` |
| `Sensor` | `value`, `truth`, `valid`, `fresh`, `channels`, `ray_ends`, настройка шума и задержки |
| `Chain` | `world.make_chain(tip)`: `forward(q)`, `jacobian(q)`, `solve_ik(...)`, `plan_line(...)`, `coordinates()` |
| `Trajectory` | `plan_point_to_point`, `plan_waypoints`, `Chain.plan_line`; `sample(t)`, `positions`, `velocities`, `times`; прогон — `follow_trajectory` |

Векторы возвращаются как `numpy`-массивы, а на вход принимаются любые последовательности из чисел. Перечисления принимают строки без учёта регистра (`"hinge"`, `"PID"`, `"lidar2d"`), а неверное имя даёт `ValueError` со списком допустимых.

Ошибки движка — `rm.RoboticsError` (наследник `RuntimeError`); неверные аргументы — `ValueError`, `IndexError`, `KeyError`.

## Как этим пользоваться

**Регуляторы.** `joint.set_controller("pid", kp=…, ki=…, kd=…)`, затем `joint.target = …` (константа) или `joint.set_step(...)`, `joint.set_sine(...)`. Для векторных команд: `world.set_joint_targets(array)` по всем приводным суставам (порядок — `world.actuated_joints`).

**Датчики.** По умолчанию они реалистичны: энкодер квантует до 4096 отсчётов на оборот, IMU шумит. Для идеального датчика передайте `encoder_counts=0` / `noise_stddev=0`. С `feeds_controller=True` регулятор замыкается на показание датчика (шум, квантование и задержка попадают в контур).

**Снимки и воспроизводимость.** `world.clone()` — независимая копия всего состояния; `world.copy_state_from(snapshot)` возвращает мир к снимку (дескрипторы остаются валидными). Одинаковый ввод даёт побитово одинаковый результат.

**Потоки.** `step`/`run` отпускают GIL, длинные прогоны не блокируют другие потоки Python.

Кинематика и планирование — в [PLANNING.md](PLANNING.md), окружение для RL — в [GYM.md](GYM.md).

## Загрузка и сохранение

```python
world = rm.load_scene("examples/manipulator_3dof.robot")
rm.save_scene(world, "copy.robot")

robot = rm.load_urdf("examples/urdf/two_link_arm.urdf", fix_base=True)
print(robot.name, robot.warnings)
world = robot.world

rm.demos.manipulator()      # встроенные примеры: pid_servo, rover, sensor_rover, noisy_pid, ...
```

## Ограничения

- Прямой управляющий момент на сустав не задаётся: приводы работают через скоростной мотор или регулятор положения/скорости (`motor_speed`, `target`). Силы и моменты на тело (`apply_force`) действуют один шаг.
- Выпуклые оболочки из Python пока не создаются (читаются из сцен и URDF).
- Камеры рендерит только редактор: у `Sensor` камеры `value` пуст.
