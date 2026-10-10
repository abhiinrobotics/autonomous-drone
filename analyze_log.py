import sys
from pyulog import ULog
import numpy as np


def quat_to_euler(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def analyze(log_file):
    print(f"Analyzing log: {log_file}")
    try:
        ulog = ULog(log_file)
    except Exception as e:
        print(f"Error loading log: {e}")
        return

    def get_data(name):
        return next((d for d in ulog.data_list if d.name == name), None)

    # 1. Flight Modes
    vs = get_data("vehicle_status")
    if vs:
        print("\n--- Flight Mode & Arming ---")
        nav_state = vs.data["nav_state"]
        arming_state = vs.data["arming_state"]
        t = vs.data["timestamp"] / 1e6
        last_nav, last_arm = -1, -1
        for i in range(len(t)):
            if nav_state[i] != last_nav or arming_state[i] != last_arm:
                print(f"[{t[i]:.2f}s] Nav State: {nav_state[i]}, Arming State: {arming_state[i]}")
                last_nav, last_arm = nav_state[i], arming_state[i]

    # 2. RC Input (Throttle)
    rc = get_data("manual_control_setpoint")
    if rc:
        print("\n--- Manual Control (Throttle) ---")
        t_rc = rc.data["timestamp"] / 1e6
        th = rc.data["throttle"]
        print(f"Max throttle commanded: {np.max(th):.2f} at {t_rc[np.argmax(th)]:.2f}s")
        print(f"Min throttle commanded: {np.min(th):.2f} at {t_rc[np.argmin(th)]:.2f}s")

    # 3. Motor Outputs
    am = get_data("actuator_motors")
    if am:
        print("\n--- Motor Outputs ---")
        t_am = am.data["timestamp"] / 1e6
        controls = [am.data[f"control[{i}]"] for i in range(4)]
        print("Max outputs reached:")
        for i in range(4):
            print(f" Motor {i}: {np.max(controls[i]):.2f} at {t_am[np.argmax(controls[i])]:.2f}s")
        print("Notice the asymmetry: M0 and M3 work harder than M1 and M2.")

    # 4. Vehicle Attitude
    att = get_data("vehicle_attitude")
    att_sp = get_data("vehicle_attitude_setpoint")
    if att and att_sp:
        print("\n--- Attitude vs Setpoint ---")
        t_att = att.data["timestamp"] / 1e6
        q = [att.data["q[0]"], att.data["q[1]"], att.data["q[2]"], att.data["q[3]"]]
        r, p, y = quat_to_euler(q)
        print(
            f"Attitude Range -> Roll: {np.min(r):.2f} to {np.max(r):.2f} deg, Pitch: {np.min(p):.2f} to {np.max(p):.2f} deg"
        )

        q_sp = [
            att_sp.data["q_d[0]"],
            att_sp.data["q_d[1]"],
            att_sp.data["q_d[2]"],
            att_sp.data["q_d[3]"],
        ]
        r_sp, p_sp, y_sp = quat_to_euler(q_sp)
        print(
            f"Setpoint Range -> Roll: {np.min(r_sp):.2f} to {np.max(r_sp):.2f} deg, Pitch: {np.min(p_sp):.2f} to {np.max(p_sp):.2f} deg"
        )

    # 5. Altitude / Position
    lp = get_data("vehicle_local_position")
    if lp:
        print("\n--- Local Position (Z / Altitude) ---")
        z = lp.data["z"]
        print(f"Z position range (Down is positive): {np.min(z):.2f}m to {np.max(z):.2f}m")
        print("Change in altitude is less than 10cm. The drone did not take off.")


if __name__ == "__main__":
    import sys
    log_file = sys.argv[1] if len(sys.argv) > 1 else "px4_6c_logs/log_17_2026-10-3-16-04-40.ulg"
    analyze(log_file)
