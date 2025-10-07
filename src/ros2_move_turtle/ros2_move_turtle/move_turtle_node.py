import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Publisher to control turtle velocity
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to turtle position
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Create a timer to publish movement commands
        self.timer = self.create_timer(0.1, self.move_turtle)

        # Store pose
        self.pose = Pose()

        # Movement velocity
        self.linear_speed = 1.0
        self.angular_speed = 1.0

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def move_turtle(self):
        cmd = Twist()

        # Check if the turtle has crossed the boundary
        if self.pose.x > 7.0 or self.pose.y > 7.0:
            # Stop movement
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            # Log the stop message only once
            if not hasattr(self, 'stopped') or not self.stopped:
                self.get_logger().info(f"Turtle stopped at x={self.pose.x:.2f}, y={self.pose.y:.2f}")
                self.stopped = True
        else:
            # Move normally (circular motion)
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_speed
            self.stopped = False

        self.cmd_pub.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
