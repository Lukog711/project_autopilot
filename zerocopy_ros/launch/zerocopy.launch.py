import os
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def check_sudo_available():
    """Check if passwordless sudo is configured"""
    try:
        result = subprocess.run(['sudo', '-n', 'true'], capture_output=True, timeout=1)
        return result.returncode == 0
    except:
        return False

def generate_launch_description():
    
    # Check sudo availability
    has_sudo = check_sudo_available()
    if not has_sudo:
        print("\n" + "="*60)
        print("⚠️  WARNING: Passwordless sudo not configured!")
        print("="*60)
        print("RT priorities and CPU affinity will be disabled.")
        print("\nTo enable, run:")
        print("  sudo visudo")
        print("\nAdd this line:")
        print("  your_username ALL=(ALL) NOPASSWD: /usr/bin/chrt, /usr/bin/taskset")
        print("="*60 + "\n")
    
    # Declare arguments
    use_shm_arg = DeclareLaunchArgument(
        'use_shm', 
        default_value='true',
        description='Enable Shared Memory (iceoryx)'
    )
    
    # Configuration (dynamic paths)
    home = os.environ.get('HOME')
    shm_config_file = os.path.join(home, 'cyclonedds_shm.xml')
    
    # Preserve environment for sudo
    ld_lib_path = os.environ.get('LD_LIBRARY_PATH', '')
    python_path = os.environ.get('PYTHONPATH', '')
    display = os.environ.get('DISPLAY', ':0')
    xauth = os.environ.get('XAUTHORITY', os.path.join(home, '.Xauthority'))
    
    # Virtual environment Python
    venv_python = os.path.join(home, 'drone_project/yolo_env/bin/python')
    if not os.path.exists(venv_python):
        print(f"⚠️  Virtual env Python not found: {venv_python}")
        print(f"   Using system Python: {sys.executable}")
        venv_python = sys.executable
    
    # RT prefixes
    if has_sudo:
        env_vars = f"LD_LIBRARY_PATH={ld_lib_path} PYTHONPATH={python_path} HOME={home} DISPLAY={display} XAUTHORITY={xauth}"
        
        # Publisher gets HIGH priority (sensor data is critical)
        rt_prefix_pub = f"sudo env {env_vars} taskset -c 0 chrt -f 80 "
        
        # AI gets LOW priority (non-critical, can drop frames)
        rt_prefix_ai = f"sudo env {env_vars} taskset -c 1,2,3 chrt -r 50 {venv_python} "
        
        print("\n✅ RT Configuration:")
        print(f"   Publisher: FIFO Priority 80; Core 0")
        print(f"   AI: RR Priority 50; Cores 1,2,3 \n")
    else:
        rt_prefix_pub = ''
        rt_prefix_ai = f'{venv_python} '
    
    # RouDi daemon 
    roudi_daemon = ExecuteProcess(
        cmd=['bash', '-c', '''
            sudo rm -rf /tmp/roudi* /dev/shm/iceoryx*
            iox-roudi
        '''],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_shm'))  
    )
    
    # Shared memory nodes
    shm_nodes = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_shm')),
        actions=[
            LogInfo(msg='Starting in SHARED MEMORY mode...'),
            
            Node(
                package='zerocopy_ros', 
                executable='video_pub', 
                name='video_publisher',
                output='screen', 
                prefix=rt_prefix_pub,
                additional_env={
                    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                    'CYCLONEDDS_URI': 'file://' + shm_config_file
                }
            ),
            
            Node(
                package='zerocopy_ros', 
                executable='yolo_sub.py', 
                name='yolo_subscriber',
                output='screen', 
                prefix=rt_prefix_ai,
                additional_env={
                    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                    'CYCLONEDDS_URI': 'file://' + shm_config_file
                }
            )
        ]
    )
    
    # Network nodes (fallback)
    net_nodes = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_shm')),
        actions=[
            LogInfo(msg='Starting in NETWORK mode (UDP)...'),
            
            Node(
                package='zerocopy_ros', 
                executable='video_pub', 
                name='video_publisher', 
                output='screen', 
                prefix=rt_prefix_pub
            ),
            
            Node(
                package='zerocopy_ros', 
                executable='yolo_sub.py', 
                name='yolo_subscriber', 
                output='screen', 
                prefix=rt_prefix_ai
            )
        ]
    )
    
    return LaunchDescription([
        use_shm_arg,
        roudi_daemon,
        shm_nodes,
        net_nodes,
        
        LogInfo(msg=''),
        LogInfo(msg='======================================'),
        LogInfo(msg='  Drone Object Detection System'),
        LogInfo(msg='======================================'),
        LogInfo(msg='Press ESC in video window to exit'),
        LogInfo(msg='======================================'),
    ])
