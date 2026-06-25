#!/usr/bin/env python3
"""Check ROS2 state and write to a log file."""
import subprocess
import os

log_path = "/home/howard/code/navigation2/ros2_check.log"
with open(log_path, "w") as f:
    f.write(f"=== ROS2 State Check at {os.popen('date').read()} ===\n\n")

    # Check processes
    f.write("--- ROS2 processes ---\n")
    for proc in ["ros2 launch", "map_server", "amcl", "fake_robot"]:
        result = subprocess.run(["pgrep", "-af", proc], capture_output=True, text=True, timeout=5)
        f.write(f"{proc}: {result.stdout.strip() or 'NOT RUNNING'}\n")

    f.write("\n--- Environment ---\n")
    for var in ["ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "FASTRTPS_DEFAULT_PROFILES_FILE", "RMW_IMPLEMENTATION"]:
        f.write(f"{var}={os.environ.get(var, '(unset)')}\n")

    f.write("\n--- /tmp/wheelchair_nav_logs/launch.log (last 10 lines) ---\n")
    log_file = "/tmp/wheelchair_nav_logs/launch.log"
    if os.path.exists(log_file):
        with open(log_file) as lf:
            lines = lf.readlines()
            f.writelines(lines[-10:])
    else:
        f.write("Log file not found\n")

    f.write("\n--- Network interfaces ---\n")
    try:
        result = subprocess.run(["ip", "-o", "link", "show"], capture_output=True, text=True, timeout=5)
        f.write(result.stdout)
    except Exception as e:
        f.write(f"Error: {e}\n")

print(f"Log written to {log_path}")
