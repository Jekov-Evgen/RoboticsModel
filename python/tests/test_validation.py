import pytest

pytest.importorskip("mujoco")
pytest.importorskip("matplotlib")


def test_engine_agrees_with_analytic_solutions_and_mujoco(capsys):
    """Runs python/validation/compare_mujoco.py; it returns non-zero when a tolerance is exceeded."""
    import importlib.util
    import pathlib

    path = pathlib.Path(__file__).resolve().parents[1] / "validation" / "compare_mujoco.py"
    spec = importlib.util.spec_from_file_location("compare_mujoco", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.main([]) == 0, capsys.readouterr().out
