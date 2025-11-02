#!/usr/bin/env python3
"""
Robot PLC Interlock Node
Controls PLC coil 8194 as interlock for another robot based on red detection status.

Logic:
- When red is FIRST detected: Set coil 8194 to LOW (0) - allow other robot
- When red is NOT detected (after Waypoint 2): Set coil 8194 to HIGH (1) - block other robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pyModbusTCP.client import ModbusClient


class RobotPLCInterlockNode(Node):
    def __init__(self):
        super().__init__('robot_plc_interlock')
        
        # Parameters
        self.declare_parameter('plc_ip', '192.168.0.13')
        self.declare_parameter('plc_port', 502)
        self.declare_parameter('unit_id', 1)
        self.declare_parameter('coil_address', 8194)
        self.declare_parameter('red_detection_topic', '/red_detected')
        self.declare_parameter('reset_after_waypoint2_topic', '/reset_interlock_after_waypoint2')
        self.declare_parameter('init_startup_topic', '/init_interlock_startup')
        self.declare_parameter('set_low_topic', '/set_interlock_low')
        self.declare_parameter('set_high_topic', '/set_interlock_high')
        
        plc_ip = self.get_parameter('plc_ip').get_parameter_value().string_value
        plc_port = self.get_parameter('plc_port').get_parameter_value().integer_value
        unit_id = self.get_parameter('unit_id').get_parameter_value().integer_value
        self.coil_address = self.get_parameter('coil_address').get_parameter_value().integer_value
        red_topic = self.get_parameter('red_detection_topic').get_parameter_value().string_value
        reset_topic = self.get_parameter('reset_after_waypoint2_topic').get_parameter_value().string_value
        init_topic = self.get_parameter('init_startup_topic').get_parameter_value().string_value
        set_low_topic = self.get_parameter('set_low_topic').get_parameter_value().string_value
        set_high_topic = self.get_parameter('set_high_topic').get_parameter_value().string_value
        
        # PLC Connection
        self.plc_client = ModbusClient(host=plc_ip, port=plc_port, unit_id=unit_id, auto_open=True)
        
        # Test connection
        if not self.plc_client.open():
            self.get_logger().error(f"Failed to connect to PLC at {plc_ip}:{plc_port}")
            raise ConnectionError(f"Failed to connect to PLC at {plc_ip}:{plc_port}")
        self.get_logger().info(f"✅ Connected to PLC at {plc_ip}:{plc_port}")
        
        # State tracking
        self.last_red_state = None
        self.red_detected_handled = False  # Track if we've handled the first red detection
        self.reset_after_waypoint2 = False  # Flag to check red after Waypoint 2
        
        # Subscribe to red detection
        self.create_subscription(
            Bool,
            red_topic,
            self.red_detection_callback,
            10
        )
        
        # Subscribe to reset signal (from C++ code after Waypoint 2)
        self.create_subscription(
            Bool,
            reset_topic,
            self.reset_callback,
            10
        )
        
        # Subscribe to startup initialization signal (from C++ code at program start)
        self.create_subscription(
            Bool,
            init_topic,
            self.init_startup_callback,
            10
        )
        
        # Subscribe to set interlock LOW signal (from C++ code when red detected for 2 seconds at Step 7.5)
        self.create_subscription(
            Bool,
            set_low_topic,
            self.set_low_callback,
            10
        )
        
        # Subscribe to set interlock HIGH signal (from C++ code when red NOT detected at Step 7.5)
        self.create_subscription(
            Bool,
            set_high_topic,
            self.set_high_callback,
            10
        )
        
        self.get_logger().info("Robot PLC Interlock Node ready")
        self.get_logger().info(f"PLC: {plc_ip}:{plc_port}, Coil: {self.coil_address}")
        self.get_logger().info(f"Red detection topic: {red_topic}")
        self.get_logger().info(f"Reset topic: {reset_topic}")
    
    def red_detection_callback(self, msg: Bool):
        """Handle red detection messages"""
        red_detected = msg.data
        
        # If this is the first red detection (transition from False/None to True)
        if red_detected and not self.red_detected_handled:
            self.get_logger().info("🔴 Red DETECTED - Setting interlock coil 8194 to LOW (0)")
            if self.set_plc_coil(self.coil_address, 0):
                self.red_detected_handled = True
                self.get_logger().info("✅ Interlock set to LOW - Other robot can proceed")
        
        # Track state for later use
        self.last_red_state = red_detected
    
    def init_startup_callback(self, msg: Bool):
        """Handle startup initialization signal from C++ code (triggered at program start)"""
        if msg.data:
            self.get_logger().info("🚀 Startup initialization signal received - Setting interlock coil 8194 to LOW (0)")
            if self.set_plc_coil(self.coil_address, 0):
                self.get_logger().info("✅ Interlock initialized to LOW - Other robot can proceed")
                # Reset state tracking for new cycle
                self.red_detected_handled = False
                self.last_red_state = None
    
    def set_low_callback(self, msg: Bool):
        """Handle set interlock LOW signal from C++ code (triggered when red detected for 2 seconds at Step 7.5)"""
        if msg.data:
            self.get_logger().info("🔴 Red detected for 2 seconds - Setting interlock coil 8194 to LOW (0)")
            if self.set_plc_coil(self.coil_address, 0):
                self.get_logger().info("✅ Interlock set to LOW - Other robot can proceed")
                # Reset state tracking
                self.red_detected_handled = False
    
    def set_high_callback(self, msg: Bool):
        """Handle set interlock HIGH signal from C++ code (triggered when red NOT detected at Step 7.5)"""
        if msg.data:
            self.get_logger().info("🟢 Red NOT detected - Setting interlock coil 8194 to HIGH (1)")
            if self.set_plc_coil(self.coil_address, 1):
                self.get_logger().info("✅ Interlock set to HIGH - Other robot blocked")
                # Reset state tracking
                self.red_detected_handled = False
    
    def reset_callback(self, msg: Bool):
        """Handle reset signal from C++ code (triggered after Waypoint 2)"""
        if msg.data:
            self.get_logger().info("🔄 Reset signal received - Checking red detection status after Waypoint 2...")
            # Give a small moment to get latest red detection state
            rclpy.spin_once(self, timeout_sec=0.2)
            
            # If red is NOT detected, set coil to HIGH (1)
            if self.last_red_state is False:
                self.get_logger().info("🟢 Red NOT detected - Setting interlock coil 8194 to HIGH (1)")
                if self.set_plc_coil(self.coil_address, 1):
                    self.get_logger().info("✅ Interlock set to HIGH - Other robot blocked")
                    self.red_detected_handled = False  # Reset for next cycle
            elif self.last_red_state is True:
                self.get_logger().warn("⚠️ Red still detected - Keeping interlock LOW")
                # Keep checking periodically via timer
                self.reset_after_waypoint2 = True
            else:
                # last_red_state is None - assume not detected
                self.get_logger().info("🟢 No red detection data - Setting interlock coil 8194 to HIGH (1)")
                if self.set_plc_coil(self.coil_address, 1):
                    self.get_logger().info("✅ Interlock set to HIGH - Other robot blocked")
                    self.red_detected_handled = False  # Reset for next cycle
    
    def check_and_reset_interlock(self):
        """Check red detection and reset interlock if red is NOT detected (called by timer)"""
        if self.reset_after_waypoint2:
            # If red is NOT detected, set coil to HIGH (1)
            if self.last_red_state is False:
                self.get_logger().info("🟢 Red NOT detected - Setting interlock coil 8194 to HIGH (1)")
                if self.set_plc_coil(self.coil_address, 1):
                    self.get_logger().info("✅ Interlock set to HIGH - Other robot blocked")
                    self.reset_after_waypoint2 = False
                    self.red_detected_handled = False  # Reset for next cycle
            elif self.last_red_state is True:
                # Red still detected, will check again next timer tick
                pass
    
    def set_plc_coil(self, coil_address: int, state: int) -> bool:
        """Set PLC coil to HIGH (1) or LOW (0)"""
        try:
            if state not in [0, 1]:
                self.get_logger().error(f"Invalid state: {state}. Must be 0 (LOW) or 1 (HIGH)")
                return False
            
            # Ensure connection is open
            if not self.plc_client.is_open:
                self.plc_client.open()
            
            result = self.plc_client.write_single_coil(coil_address, state)
            
            if not result:
                self.get_logger().error(f"Failed to write to coil {coil_address}")
                return False
            
            state_str = "HIGH" if state == 1 else "LOW"
            self.get_logger().info(f"✅ Set coil {coil_address} to {state_str} (value: {state})")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Exception setting coil {coil_address}: {e}")
            return False
    
    def destroy_node(self):
        """Clean up PLC connection"""
        if self.plc_client:
            self.plc_client.close()
            self.get_logger().info("🔌 Disconnected from PLC")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPLCInterlockNode()
    
    try:
        # Create a timer to periodically check reset status
        def check_reset_timer():
            if node.reset_after_waypoint2:
                node.check_and_reset_interlock()
        
        timer = node.create_timer(0.5, check_reset_timer)  # Check every 0.5 seconds
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

