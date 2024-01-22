import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
import time

class RobotController:
    def __init__(self, rate, model_name):
        self.rate = rate  # rate in Hz
        self.model_name = model_name
        self.last_print_time = 0

        # Publishers for each wheel
        self.front_left_wheel_pub = rospy.Publisher('/ford_robot/front_left_wheel_position_controller/command', Float64, queue_size=10)
        self.front_right_wheel_pub = rospy.Publisher('/ford_robot/front_right_wheel_position_controller/command', Float64, queue_size=10)
        self.rear_left_wheel_pub = rospy.Publisher('/ford_robot/rear_left_wheel_position_controller/command', Float64, queue_size=10)
        self.rear_right_wheel_pub = rospy.Publisher('/ford_robot/rear_right_wheel_position_controller/command', Float64, queue_size=10)

    def callback(self, data):
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0 / self.rate:
            try:
                index = data.name.index(self.model_name)
                pose = data.pose[index]
                twist = data.twist[index]

                print("Position of the {}:".format(self.model_name))
                print("x: {}, y: {}, z: {}".format(pose.position.x, pose.position.y, pose.position.z))
                print("twist linear x: {}".format(twist.linear.x))

                self.publish_wheel_commands()

                self.last_print_time = current_time
            except ValueError:
                rospy.logwarn("Model {} not found".format(self.model_name))

    def publish_wheel_commands(self):
        # Example: Set wheel velocities
        velocity = Float64()
        velocity = 2.0  # Set your desired wheel velocity here

        self.front_left_wheel_pub.publish(velocity)
        self.front_right_wheel_pub.publish(velocity)
        self.rear_left_wheel_pub.publish(velocity)
        self.rear_right_wheel_pub.publish(velocity)

def main():
    rospy.init_node('ford_robot_controller')

    # Set the control rate (e.g., 1 Hz) and the name of your robot model
    controller = RobotController(rate=60, model_name='ford_robot')
    rospy.Subscriber('/gazebo/model_states', ModelStates, controller.callback)

    rospy.spin()

if __name__ == '__main__':
    print("Starting ROS node...")
    main()