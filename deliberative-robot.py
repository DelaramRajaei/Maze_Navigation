import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import Astar


class Deliberative:

    def __init__(self):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.bumper = {}
        self.ranges = {}
        self.path = {}
        self.count = 0
        self.deliberative_publisher

    def callback(self, msg):
        self.position["x"] = msg.pose.pose.position.x
        self.position["y"] = msg.pose.pose.position.y
        self.position["z"] = msg.pose.pose.position.z
        if msg.pose.pose.position.x == 0 and msg.pose.pose.position.y == 0 and msg.pose.pose.position.z == 0:
            self.count += 1

    def callback_message(self, subscribedData):
        rospy.loginfo('Subscribed: ' + subscribedData.data)
        string = str(subscribedData.data)
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        # data = bumper-ranges
        str_position = self.position["x"], self.position["y"], self.position["z"]
        data = string.split("-")
        self.bumper[str_position] = data[0]
        # data[1:4] = right#center#left
        range = data.split("#")
        self.ranges[str_position] = range[1:4]

    def sensing(self):
        while self.count != 2:
            rospy.init_node('deliberative_controller', anonymous=False)
            rospy.Subscriber("velocity_topic", String, self.callback_message)

    def get_end_position(self):
        return input("Goal position: ")

    def planing(self):
        astar = Astar()
        end = self.get_end_position()
        start = (0, 0, 0)
        self.path = astar.astar_search(self.ranges, start, end)

    def acting(self):
        x_vel = []
        z_vel = []

        for i in range(len(self.path)):
            x_vel.append(self.path("linearX"))
            z_vel.append(self.path("angularZ"))
            self.deliberative_msg.linear.x = self.path("linearX")
            self.deliberative_msg.angular.z = self.path("angularZ")
            self.deliberative_publisher.publish(self.deliberative_msg)

    def main(self):
        try:
            self.deliberative_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.deliberative_msg = Twist()
            # Sense
            self.sensing()
            # Plan
            self.planing()
            # Act
            self.acting()

        except rospy.ROSInterruptException:
            pass

    if __name__ == '__main__':
        main()



