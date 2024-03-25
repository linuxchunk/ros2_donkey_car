#!/usr/bin/python3

"""
Class for low level control of our car. It assumes ros-12cpwmboard has been
installed
"""
import rclpy
from rclpy.node import Node
from i2cpwm_board_msgs.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time


class ServoConvert():
    def __init__(self, id=1, center_value=333, range=90, direction=1):
        self.value = 0.0
        self.value_out = center_value
        self._center = center_value
        self._range = range
        self._half_range = 0.5 * range
        self._dir = direction
        self.id = id

        #--- Convert its range in [-1, 1]
        self._sf = 1.0 / self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value = value_in
        self.value_out = int(self._dir * value_in * self._half_range + self._center)
        print(self.id, self.value_out)
        return self.value_out


class DkLowLevelCtrl(Node):
    def __init__(self):
        super().__init__()
        self.get_logger().info("Setting Up the Node...")

        self.actuators = {}
        self.actuators['throttle'] = ServoConvert(id=1)
        self.actuators['steering'] = ServoConvert(id=2, direction=1)  #-- positive left
        self.get_logger().info("> Actuators correctly initialized")

        self._servo_msg = ServoArray()
        for _ in range(2):
            self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array = self.create_publisher(ServoArray, "/servos_absolute", 1)
        self.get_logger().info("> Publisher correctly initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist = self.create_subscription(Twist, "/cmd_vel", self.set_actuators_from_cmdvel, 1)
        self.get_logger().info("> Subscriber correctly initialized")

        #--- Get the last time we got a command
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5

        self.get_logger().info("Initialization complete")

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        self.get_logger().info("Got a command v = %2.1f  s = %2.1f" % (message.linear.x, message.angular.z))
        self.send_servo_msg()

    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        self.get_logger().info("Setting actuators to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.items():
            self._servo_msg.servos[servo_obj.id - 1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id - 1].value = servo_obj.value_out
            self.get_logger().info("Sending to %s command %d" % (actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        print(time.time() - self._last_time_cmd_rcv)
        return time.time() - self._last_time_cmd_rcv < self._timeout_s

    def run(self):

        #--- Set the control rate
        rate = self.create_rate(10)

        while rclpy.ok():
            print(self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    dk_llc = DkLowLevelCtrl()
    dk_llc.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
