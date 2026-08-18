import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def check_rt_capable():
    try:
        result = subprocess.run(["sudo", "-n", "true"], capture_output=True, timeout=1)
        if result.returncode != 0: return False
        with open("/proc/sys/kernel/sched_rt_runtime_us", "r") as f:
            rt_runtime = int(f.read().strip())
        return rt_runtime != 0
    except Exception: return False

def generate_launch_description():
    livox_config = os.path.join(get_package_share_directory("livox_ros_driver2"), "config", "MID360_config.json")
    shm_config = os.path.join(os.environ.get("HOME"), "cyclonedds_shm.xml")

    has_rt = check_rt_capable()
    
    env_vars = f"LD_LIBRARY_PATH={os.environ.get('LD_LIBRARY_PATH', '')} HOME={os.environ.get('HOME')} DISPLAY={os.environ.get('DISPLAY', ':0')} XAUTHORITY={os.environ.get('XAUTHORITY', '')} WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY', '')} XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')}"

    if has_rt:
        rt_prefix_critical = f"sudo env {env_vars} taskset -c 0 chrt -f 80 "
        rt_prefix_logic    = f"sudo env {env_vars} taskset -c 1,2 chrt -r 50 "
        rt_prefix_ui       = f"sudo env {env_vars} taskset -c 3 "
    else:
        rt_prefix_critical = rt_prefix_logic = rt_prefix_ui = ""

    roudi_daemon = ExecuteProcess(
        cmd=["bash", "-c", "sleep 0.5 && sudo rm -rf /tmp/roudi* /dev/shm/iceoryx* && iox-roudi"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("CYCLONEDDS_URI", f"file://{shm_config}"),
        roudi_daemon,

        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_lidar_publisher",
             parameters=[{"user_config_path": livox_config, "xfer_format": 1}], prefix=rt_prefix_critical, output="screen"),
        Node(package="autopilot", executable="closest_obstacle_node", name="lidar_distance", prefix=rt_prefix_critical, output="screen"),
        Node(package="autopilot", executable="safe_braking_node", name="safe_braking_node", prefix=rt_prefix_logic, output="screen"),
        Node(package="autopilot", executable="health_monitor_node", name="watchdog", prefix=rt_prefix_logic, output="screen"),
        Node(package="autopilot", executable="decision_node", name="ui_dashboard", prefix=rt_prefix_ui, output="screen"),
    ])
