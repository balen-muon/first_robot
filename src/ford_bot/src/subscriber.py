import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Twist
import time

class RobotController:
    def __init__(self, rate, model_name):
        self.rate = rate  # rate in Hz
        self.model_name = model_name
        self.last_print_time = 0
        self.state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

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

                # Update and publish the new state
                self.publish_new_state(pose, twist)

                self.last_print_time = current_time
            except ValueError:
                rospy.logwarn("Model {} not found".format(self.model_name))

    def publish_new_state(self, pose, twist):
        new_state = ModelState()
        new_state.model_name = self.model_name
        new_state.pose = pose
        new_state.twist = twist

        # Modify this to change the robot's behavior
        new_state.twist.linear.x += 0.1  # Example: Increment linear x velocity

        self.state_pub.publish(new_state)

def main():
    rospy.init_node('gazebo_robot_controller')

    # Set the control rate (e.g., 1 Hz) and the name of your robot model
    controller = RobotController(rate=1, model_name='ford_robot')
    rospy.Subscriber('/gazebo/model_states', ModelStates, controller.callback)

    rospy.spin()

if __name__ == '__main__':
    print("Starting ROS node...")
    main()