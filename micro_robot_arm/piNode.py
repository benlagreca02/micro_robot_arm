import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import os

PI_NAME = "rpi4"

# only include pi stuff on pi
system = os.uname().nodename
if(system == PI_NAME):
    from adafruit_servokit import ServoKit


class JointSubscriber(Node):
    #! Map joint names to channel numbers. This is something we should specify in the urdf or a separate config file.
    #! Should also include addition tunning parameters, for now just min/max pulse width
    SERVO_MAP = {
        'base-to-circle':0,
        's0-to-s1':1,
        's1-to-s2':2,
        's2-to-s3':3,
        'gripper':4
    }

    def __init__(self):
        super().__init__('pi_joint_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.jointCallback,
            10)
        self.subscription  # prevent unused variable warning
        self.previousVal = None

        if(system == PI_NAME):
            self.kit = ServoKit(channels=16)
            #TODO servo tunning
            # self.kit.servo[0].set_pulse_width_range(1000, 2000) 



    def jointCallback(self, msg : JointState):
        joint_positions = [math.degrees(x) for x in msg.position]
        joint_positions = dict(zip(msg.name, joint_positions))
        if(joint_positions == self.previousVal):
            return
        
        self.previousVal = joint_positions

        self.get_logger().info(f'Received joint positions: {joint_positions}')

        if(system == PI_NAME):
            for joint, jointPos in joint_positions.items():
                if joint in JointSubscriber.SERVO_MAP:
                    self.get_logger().info(f"setting '{joint}'({JointSubscriber.SERVO_MAP[joint]}) to {jointPos}")
                    self.kit.servo[JointSubscriber.SERVO_MAP[joint]].angle = jointPos + 90

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