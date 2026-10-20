"""Launch the unchanged v0.1 Bandit MuJoCo expressive wave demonstration."""
from pathlib import Path
import runpy

PROJECT = Path(__file__).resolve().parents[1]
runpy.run_path(str(PROJECT / "bandit_tools" / "bandit_final_mujoco_wave.py"), run_name="__main__")
