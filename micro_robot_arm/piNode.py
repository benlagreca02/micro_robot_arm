import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointSubscriber(Node):
    def __init__(self):
        super().__init__('pi_joint_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.jointCallback,
            10)
        self.subscription  # prevent unused variable warning

    def jointCallback(self, msg : JointState):
        joint_positions = dict(zip(msg.name, msg.position))
        self.get_logger().info(f'Received joint positions: {joint_positions}')


def main(args=None):
    rclpy.init(args=args)
    
    jointSubscriber = JointSubscriber()

    rclpy.spin(jointSubscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    jointSubscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()