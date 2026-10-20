"""Static, offline repo validation: safe paths, meshes and files."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
urdf = ROOT / 'bandit_description/bandit_final_v2.urdf'
mjcf = ROOT / 'RP1_description/mjcf/robot_24dof.xml'
controller = ROOT / 'bandit_tools/bandit_final_mujoco_wave.py'
missing = []
for mesh in ET.parse(urdf).getroot().iter('mesh'):
    name = mesh.get('filename')
    if name and not (urdf.parent / name.replace('\\','/')).is_file():
        missing.append(('URDF', name))
root = ET.parse(mjcf).getroot()
comp = root.find('compiler')
meshdir = comp.get('meshdir', '') if comp is not None else ''
for mesh in root.iter('mesh'):
    name = mesh.get('file')
    if name and not (mjcf.parent / meshdir / name).is_file():
        missing.append(('MJCF', name))
assert (ROOT / 'bandit_description/meshes/bandit_torso_visual.obj').is_file(), 'Missing converted MuJoCo torso OBJ'
assert (ROOT / 'bandit_description/meshes/textures/bandit_head_basecolor.png').is_file(), 'Missing atlas'
assert controller.is_file(), 'Missing demo controller'
assert not missing, f'Missing mesh dependencies: {missing}'
for p in [controller, ROOT / 'examples/run_bandit.py']:
    text = p.read_text(encoding='utf-8-sig')
    compile(text, str(p), 'exec')
    assert not re.search(r'[A-Za-z]:[\\/]Users[\\/]', text), f'Hardcoded user path in {p}'
print('PASS: URDF/MJCF refs, converted OBJ, texture, Python syntax and personal-path checks')
