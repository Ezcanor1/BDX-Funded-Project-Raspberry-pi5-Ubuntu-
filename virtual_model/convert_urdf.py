# convert_urdf.py
import mujoco

# --- Make sure these paths are correct ---
urdf_input_path = './robot/robot.urdf'
mjcf_output_path = './robot/robot.mjcf'

print(f"Attempting to load URDF: {urdf_input_path}")

try:
    # 1. Load the URDF file. This triggers MuJoCo's compiler.
    model = mujoco.MjModel.from_xml_path(urdf_input_path)
    print("SUCCESS: URDF file was parsed successfully.")

    # 2. Save the compiled model as a native MJCF file.
    mujoco.mj_saveLastXML(mjcf_output_path, model)
    print(f"SUCCESS: Model converted and saved to {mjcf_output_path}")
    print("\nYou can now proceed to the next step.")

except ValueError as e:
    print(f"\n--- CONVERSION FAILED ---")
    print(f"An error occurred while parsing the URDF file: {e}")
    print("\nThis means there is an issue within your 'robot.urdf' file that needs to be fixed.")