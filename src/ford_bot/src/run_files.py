import subprocess
import time
import rospy
from geometry_msgs.msg import Twist

def launch_ros_file(launch_file_path):
    """ Launches a ROS launch file. """
    command = "roslaunch {}".format(launch_file_path)
    process = subprocess.Popen(command.split())

    return process

def callback(data):
    """ Callback function for the /cmd_vel topic. """
    print("HELLO HELLO HELLO HELLO HELLO HELLO", data)

def main():
    # Path to your launch file
    launch_file = 'ford_bot gazebo.launch'
    sub = rospy.Subscriber('/cmd_vel', Twist, callback)
    print("Starting ROS launch file...")
    process = launch_ros_file(launch_file)

    # Keep the script alive or do other stuff here
    try:
        while True:
            time.sleep(1)
            # You can add code here that runs concurrently with your launch file
    except KeyboardInterrupt:
        print("Shutting down ROS launch...")

        # Terminate the process
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    

if __name__ == '__main__':
    main()