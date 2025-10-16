# test_robot.py (modified to load the MJCF)
import mujoco
import mujoco.viewer

# Load the NEWLY CREATED .mjcf file
mjcf_path = 'robot.mjcf'

try:
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    print("SUCCESS: MJCF model loaded. Launching viewer.")
    mujoco.viewer.launch(model, data)
except Exception as e:
    print(f"ERROR: Could not load the MJCF file. {e}")