import rclpy
import serial
import json
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateUploader(Node):

    def __init__(self):
        super().__init__('joint_state_uploader')
        
        self.chunk_size = 16
        self.timer_period = 0.005
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.timer = self.create_timer(
            self.timer_period,
            self.send_data_to_arduino,
        )
        self.ser = serial.Serial('/dev/ttyACM0',500000)
        self.incoming_joint_state = ""
        self.previous_joint_state = ""
        self.outgoing_msg = ""
        self.serial_msg = {}
        self.chunk_number = 0
        self.is_busy = False
        
    def listener_callback(self, joint_state):
        if (self.is_busy):
            return
        
        self.incoming_joint_state = joint_state
        # this is such a fucking hack im so sorry
        if (self.previous_joint_state == ""):
            self.previous_joint_state = joint_state
        
        if (self.incoming_joint_state.position == self.previous_joint_state.position):
            return
        else:
            self.get_logger().info("Setting new joint state to send!")
            self.previous_joint_state = self.incoming_joint_state
        
        servo_joints = [
            "DOF_1",
            "DOF_2",
            "DOF_3",
            "DOF_4",
            "DOF_5",
            "GRIPPER",
        ]
        
        for i, x in enumerate(self.incoming_joint_state.name):
            name = self.incoming_joint_state.name[i]  
            position = self.incoming_joint_state.position[i]
            if (name not in servo_joints):
                continue
            self.serial_msg[name] = int(position/math.pi*255)+1
            
        self.outgoing_msg = json.dumps(self.serial_msg)
        self.is_busy = True
        
    def send_data_to_arduino(self):
        if (self.is_busy == False):
            return

        chunk = self.outgoing_msg[self.chunk_number*self.chunk_size:(self.chunk_number+1)*self.chunk_size]
        
        if (chunk == ""):
            self.get_logger().info("Successfully sent " + self.outgoing_msg)
            self.chunk_number = 0
            self.is_busy = False
            return

        self.ser.write(
            bytearray(
                chunk,
                "utf-8"
            )
        )
        self.chunk_number += 1
                 
def main(args=None):
    rclpy.init(args=args)

    joint_state_uploader = JointStateUploader()

    rclpy.spin(joint_state_uploader)

    joint_state_uploader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()