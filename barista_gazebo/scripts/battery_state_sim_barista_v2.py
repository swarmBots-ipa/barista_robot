#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

"""
# Constants are chosen to match the enums in the linux kernel
# defined in include/linux/power_supply.h as of version 3.7
# The one difference is for style reasons the constants are
# all uppercase not mixed case.

# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

std_msgs/Header  header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
float32 voltage          # Voltage in Volts (Mandatory)
float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
float32[] cell_temperature # An array of individual cell temperatures for each cell in the pack
                           # If individual temperatures unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number


"""


class BatterySim(Node):

    def __init__(self, battery_update_period=0.5, nominal_current=5.0):
        super().__init__('sub_node')

        self.group1 = ReentrantCallbackGroup()

        self.declare_parameter('barista_name', "barista_GENERIC")

        self._battery_update_period = battery_update_period
        self.original_nominal_current = nominal_current
        self._nominal_current = nominal_current

        self.getting_params()

        self.battery_msg = BatteryState()

        name_of_batery_topic = "/" + self.barista_name + "/" + 'battery_state'

        self.battery_msg_pub = self.create_publisher(
            BatteryState, name_of_batery_topic, 1)

        self.init_battery_values()

        self.timer = self.create_timer(
            self._battery_update_period,
            self.timer_callback,
            callback_group=self.group1)

        name_of_update_batery_topic = "/" + self.barista_name + \
            "/" + 'update_battery_charge_state'
        self.subscriber = self.create_subscription(
            String,
            name_of_update_batery_topic,
            self.bat_state_listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group1)

    def bat_state_listener_callback(self, msg):
        self.get_logger().info('I received as Charging State: "%s"' % str(msg))
        # We check its vallid:
        if msg.data == "DISCHARGING" or msg.data == "CHARGING":
            self._battery_charge_state = msg.data
            self._nominal_current = self.original_nominal_current
        else:
            if msg.data == "FAST_DISCHARGING":
                self._battery_charge_state = "DISCHARGING"
                self._nominal_current = 10.0
                self.get_logger().warning("FAST DISCHARGE STARTED")
            elif msg.data == "FAST_CHARGING":
                self._battery_charge_state = "CHARGING"
                self._nominal_current = 10.0
                self.get_logger().warning("FAST CHARGE STARTED")
            elif "LOW_BATTERY_SET" in msg.data:
                bat_value = float(msg.data.split("-")[1])/100
                self._battery_charge_state = "DISCHARGING"
                self.battery_msg.charge = self.battery_msg.design_capacity * bat_value
                self._nominal_current = self.original_nominal_current
                self.get_logger().warning("FAST DISCHARGE STARTED")
            else:
                self.get_logger().error("############### CHARGING STATE ERROR ==="+str(msg.data))

    def getting_params(self):

        self.barista_name = self.get_parameter(
            'barista_name').get_parameter_value().string_value

        self.get_logger().info("############### barista_name ==="+str(self.barista_name))

    def timer_callback(self):
        self.update_pub_battery_values(self._battery_update_period)

    def init_battery_values(self):
        """
        We initialise the battery values fresh.
        """
        # We start discharging
        self._battery_charge_state = "DISCHARGING"

        self.battery_msg.header.stamp = self.get_clock().now().to_msg()
        self.battery_msg.header.frame_id = "base_link"

        self.battery_msg.voltage = random.uniform(5.0, 10.0)
        self.battery_msg.temperature = random.uniform(25.0, 26.0)
        # We are discharging all the time in V1

        if self._battery_charge_state == "CHARGING":
            self.battery_msg.current = 1.0*self._nominal_current
        else:
            self.battery_msg.current = -1.0*self._nominal_current

        # This means 100 Ah = 5 Amps X/for 20 hours
        self.battery_msg.capacity = 110.0
        self.battery_msg.design_capacity = 150.0

        Max_Time_Working = self.battery_msg.design_capacity / \
            abs(self.battery_msg.current)

        self._max_charge = self._nominal_current * Max_Time_Working
        self._min_charge = 0.0
        self.battery_msg.charge = self._max_charge

        self.battery_msg.percentage = (
            self.battery_msg.charge / self.battery_msg.design_capacity)

        # Status

        self.battery_msg.power_supply_health = self.battery_msg.POWER_SUPPLY_HEALTH_GOOD
        self.battery_msg.power_supply_technology = self.battery_msg.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_msg.present = True

        self.num_of_sim_cells = 10
        self.battery_msg.cell_voltage = [
            self.battery_msg.voltage]*self.num_of_sim_cells
        self.battery_msg.cell_temperature = [
            self.battery_msg.temperature]*self.num_of_sim_cells

        self.battery_msg.location = "SIM-SLOT-1"
        self.battery_msg.serial_number = "TheConstructs-" + self.barista_name

        self.battery_msg_pub.publish(self.battery_msg)

    def update_pub_battery_values(self, delta_time=0.1):
        """
        In this first version, the battery levels are updated by time and never recharge.
        """

        self.battery_msg.header.stamp = self.get_clock().now().to_msg()
        # We simulate that they vary
        self.battery_msg.voltage = random.uniform(5.0, 10.0)
        self.battery_msg.temperature = random.uniform(25.0, 26.0)
        # Status
        self.battery_msg.cell_voltage = [
            self.battery_msg.voltage]*self.num_of_sim_cells
        self.battery_msg.cell_temperature = [
            self.battery_msg.temperature]*self.num_of_sim_cells
        # We are discharging all the time in V1

        if self._battery_charge_state == "CHARGING":
            self.battery_msg.current = 1.0*self._nominal_current
            self.battery_msg.power_supply_status = self.battery_msg.POWER_SUPPLY_STATUS_CHARGING
        else:
            self.battery_msg.current = -1.0*self._nominal_current
            self.battery_msg.power_supply_status = self.battery_msg.POWER_SUPPLY_STATUS_DISCHARGING

        capacity_delta = self.battery_msg.current * delta_time
        self.battery_msg.charge += capacity_delta

        # We sure we dont exceed limits in charge
        if self.battery_msg.charge >= self._max_charge:
            self.battery_msg.charge = self._max_charge
            self.battery_msg.power_supply_status = self.battery_msg.POWER_SUPPLY_STATUS_FULL
        elif self.battery_msg.charge <= self._min_charge:
            self.battery_msg.charge = self._min_charge
            self.battery_msg.power_supply_status = self.battery_msg.POWER_SUPPLY_STATUS_UNKNOWN

        self.battery_msg.percentage = (
            self.battery_msg.charge / self.battery_msg.design_capacity)

        self.battery_msg_pub.publish(self.battery_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        battery_state_node = BatterySim(battery_update_period=0.1,
                                        nominal_current=0.01)

        num_threads = 2
        executor = MultiThreadedExecutor(num_threads=num_threads)
        executor.add_node(battery_state_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            battery_state_node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
