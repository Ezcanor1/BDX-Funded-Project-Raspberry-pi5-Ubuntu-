# run_simple_test.py
import mujoco
import mujoco.viewer
import time

# Load the simple test file
# This path is relative to the 'testing' folder
xml_path = './robot/simple_test.xml'

print(f"Attempting to load: {xml_path}")

try:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading XML: {e}")
    exit()

# Launch the viewer
print("Success! Launching viewer...")
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)