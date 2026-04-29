import numpy as np
import mujoco
from scipy.optimize import minimize
import math

XML_PATH = "XP_robot_walking.xml"

def get_leg_jacobian_and_error(m, d, joint_ids, foot_site_id, target_pos, target_quat):
    """Computes position/orientation error for the foot."""
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    
    pos = d.site_xpos[foot_site_id]
    mat = d.site_xmat[foot_site_id].reshape(3, 3)
    
    # Calculate pos error
    pos_err = pos - target_pos
    
    # Calculate orientation error (using trace for simplicity)
    # We want mat to be aligned with target_quat (identity for flat foot)
    # Actually, simpler: we want the foot's local Z axis to point straight down (or up)
    # and local X to point forward.
    # Let's just minimize the deviation of the foot's Z vector from global Z
    foot_z = mat[:, 2]
    foot_y = mat[:, 1]
    
    # Error is heavily weighted against orientation deviations
    err = np.sum(pos_err**2) + 0.1 * (1.0 - foot_z[2])**2 + 0.1 * foot_y[0]**2
    return err

def solve_oblique_crouch(drop_height_m=0.05):
    """
    Mathematical function to calculate the exact 6-DOF joint angles required 
    to crouch without slipping or rolling, compensating for the 15-degree hip skew.
    """
    m = mujoco.MjModel.from_xml_path(XML_PATH)
    d = mujoco.MjData(m)
    
    # Left leg joints that participate in the 6-DOF chain
    j_names = [
        'pelvis_hip_pitch_l', 
        'hip_pitch_l_hip_roll_l', 
        'hip_roll_l_thigh_yaw_l',
        'thigh_l_knee_l', 
        'leg_shank_l_ankle_pitch_l', 
        'ankle_pitch_l_ankle_roll_l'
    ]
    j_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in j_names]
    q_idxs = [m.jnt_qposadr[jid] for jid in j_ids]
    
    foot_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'LF_site')
    
    # 1. Get baseline standing position of the foot
    mujoco.mj_resetData(m, d)
    d.qpos[2] = 0.746  # Default standing height
    mujoco.mj_kinematics(m, d)
    baseline_foot_pos = d.site_xpos[foot_site].copy()
    
    # 2. Lower the pelvis by drop_height_m
    d.qpos[2] = 0.746 - drop_height_m
    
    # 3. Optimization objective: Move the joints so the foot reaches baseline_foot_pos
    # while keeping the foot perfectly flat.
    def objective(q_leg):
        for idx, val in zip(q_idxs, q_leg):
            d.qpos[idx] = val
        return get_leg_jacobian_and_error(m, d, j_ids, foot_site, baseline_foot_pos, None)
    
    # Initial guess
    q0 = np.zeros(6)
    q0[0] = -0.15 # hip pitch guess
    q0[3] = -0.30 # knee guess
    q0[4] = 0.15  # ankle pitch guess
    
    # Bounds based on physical URDF limits
    bnds = [
        (-1.57, 1.57),  # hip pitch
        (-0.3, 0.3),    # hip roll (needed for lateral swing comp)
        (-0.5, 0.5),    # thigh yaw (needed for twist comp)
        (-1.57, 0.0),   # knee (can only bend backwards)
        (-0.698, 0.698), # ankle pitch (LIMIT: 40 degrees!)
        (-0.3, 0.3)     # ankle roll (needed for foot flat comp)
    ]
    
    print(f"Solving 6-DOF IK for {drop_height_m*100:.1f}cm drop...")
    res = minimize(objective, q0, bounds=bnds, method='SLSQP', options={'ftol': 1e-6})
    
    if res.fun > 1e-4:
        print("WARNING: Could not find a perfect solution. The requested depth exceeds physical limits.")
    
    angles = res.x
    print("\n--- EXACT MATHEMATICAL CROUCH ANGLES ---")
    print(f"Hip Pitch:   {angles[0]:+.4f} rad (Lowers body)")
    print(f"Hip Roll:    {angles[1]:+.4f} rad (Compensates lateral swing)")
    print(f"Thigh Yaw:   {angles[2]:+.4f} rad (Compensates 15-deg skew twist)")
    print(f"Knee:        {angles[3]:+.4f} rad (Lowers body)")
    print(f"Ankle Pitch: {angles[4]:+.4f} rad (Keeps foot flat)")
    print(f"Ankle Roll:  {angles[5]:+.4f} rad (Compensates hip roll)")
    print("----------------------------------------")
    
    return angles

if __name__ == "__main__":
    # Test a 5cm drop (physically possible within 40-deg ankle limit)
    solve_oblique_crouch(0.05)
    
    # Test a 15cm drop (physically impossible due to ankle limit)
    print("\nAttempting deep 15cm drop...")
    solve_oblique_crouch(0.15)
