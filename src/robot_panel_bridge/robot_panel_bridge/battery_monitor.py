"""Publishes the host computer's (the robot's laptop) battery for the web panel.

A browser can't read the host battery itself -- the panel is also opened from
other machines on the LAN (http://<robot-ip>:8000/), where navigator.getBattery
would report the *viewer's* battery. So this node reads the kernel's
/sys/class/power_supply/<battery> (visible read-only inside the container) and
publishes it as sensor_msgs/BatteryState on /host/battery, which the panel
subscribes to through rosbridge.

Kernel drivers report either charge_* (uAh) + current_now (uA) or energy_*
(uWh) + power_now (uW); both are handled. Values are converted to SI units
(V, A, Ah). current is negative while discharging, per the BatteryState
convention.
"""
import glob
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

POWER_SUPPLY_DIR = '/sys/class/power_supply'
TOPIC = '/host/battery'

STATUS_MAP = {
    'Charging': BatteryState.POWER_SUPPLY_STATUS_CHARGING,
    'Discharging': BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
    'Not charging': BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
    'Full': BatteryState.POWER_SUPPLY_STATUS_FULL,
}


def find_battery():
    for path in sorted(glob.glob(os.path.join(POWER_SUPPLY_DIR, '*'))):
        if read_str(path, 'type') == 'Battery':
            return path
    return None


def read_str(path, name):
    try:
        with open(os.path.join(path, name)) as f:
            return f.read().strip()
    except OSError:
        return None


def read_micro(path, name):
    """sysfs power_supply values are integers in micro-units (uV, uA, uAh, uWh, uW)."""
    value = read_str(path, name)
    try:
        return int(value) / 1e6
    except (TypeError, ValueError):
        return None


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self._battery = self.declare_parameter('battery_path', '').value or find_battery()
        self._pub = self.create_publisher(BatteryState, TOPIC, 10)
        self.create_timer(self.declare_parameter('period_sec', 1.0).value, self._publish)
        if self._battery:
            self.get_logger().info(f'Publishing {self._battery} on {TOPIC}')
        else:
            self.get_logger().warn(f'No battery found under {POWER_SUPPLY_DIR}; nothing will be published')

    def _publish(self):
        if not self._battery:
            return
        path = self._battery
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.present = read_str(path, 'present') == '1'
        msg.power_supply_status = STATUS_MAP.get(
            read_str(path, 'status'), BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
        msg.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION if read_str(path, 'technology') == 'Li-ion'
            else BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)

        voltage = read_micro(path, 'voltage_now')
        current = read_micro(path, 'current_now')
        charge = read_micro(path, 'charge_now')
        charge_full = read_micro(path, 'charge_full')
        charge_full_design = read_micro(path, 'charge_full_design')

        # energy_*/power_now flavour: convert Wh/W to Ah/A through the voltage.
        if current is None and voltage:
            power = read_micro(path, 'power_now')
            current = power / voltage if power is not None else None
        if charge is None and voltage:
            to_ah = lambda wh: wh / voltage if wh is not None else None  # noqa: E731
            charge = to_ah(read_micro(path, 'energy_now'))
            charge_full = to_ah(read_micro(path, 'energy_full'))
            charge_full_design = to_ah(read_micro(path, 'energy_full_design'))

        nan = float('nan')
        msg.voltage = voltage if voltage is not None else nan
        msg.temperature = nan
        msg.current = nan
        if current is not None:
            # Some drivers already sign current_now; normalise to BatteryState's convention.
            discharging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.current = -abs(current) if discharging else abs(current)
        msg.charge = charge if charge is not None else nan
        msg.capacity = charge_full if charge_full is not None else nan
        msg.design_capacity = charge_full_design if charge_full_design is not None else nan

        capacity_pct = read_str(path, 'capacity')
        msg.percentage = int(capacity_pct) / 100.0 if capacity_pct and capacity_pct.isdigit() else nan

        msg.location = 'host'
        msg.serial_number = read_str(path, 'serial_number') or ''
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
