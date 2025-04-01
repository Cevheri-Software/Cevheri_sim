#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleOdometry
from std_msgs.msg import Header
import numpy as np
import time

class UAVControl(Node):
    def __init__(self):
        super().__init__('uav_control')
        self.get_logger().info('UAV Control Node Started')
        
        # Publisher for vehicle commands
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
            
        # Publisher for offboard control mode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
            
        # Publisher for trajectory setpoints
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
            
        # Subscriber for vehicle status
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
            
        # Subscriber for vehicle odometry
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, 10)
        
        # Vehicle status
        self.vehicle_status = VehicleStatus()
        self.vehicle_odometry = VehicleOdometry()
        
        # Create a timer to publish control commands
        self.control_timer = self.create_timer(0.1, self.control_timer_callback)
        
        # Offboard control parameters
        self.offboard_setpoint_counter = 0
        self.is_armed = False
        self.is_offboard_mode = False
        
    def vehicle_status_callback(self, msg):
        """
        Callback function for vehicle status updates
        """
        self.vehicle_status = msg
        self.is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.is_offboard_mode = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
    def vehicle_odometry_callback(self, msg):
        """
        Callback function for vehicle odometry updates
        """
        self.vehicle_odometry = msg
        
    def control_timer_callback(self):
        """
        Callback function for the control timer
        """
        # Publish offboard control mode
        self.publish_offboard_control_mode()
        
        # Publish the trajectory setpoint
        self.publish_trajectory_setpoint()
        
        # Check if we need to switch to offboard mode and arm
        if self.offboard_setpoint_counter == 10:
            self.get_logger().info("Setting Offboard mode and arming")
            self.arm()
            self.engage_offboard_mode()
            
        # Increment the setpoint counter
        self.offboard_setpoint_counter += 1
        
    def arm(self):
        """
        Send an arm command to the vehicle
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")
        
    def disarm(self):
        """
        Send a disarm command to the vehicle
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")
        
    def engage_offboard_mode(self):
        """
        Switch to offboard mode
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode engaged")
        
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """
        Publish a vehicle command
        """
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        
    def publish_offboard_control_mode(self):
        """
        Publish the offboard control mode
        """
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)
        
    def publish_trajectory_setpoint(self):
        """
        Publish a trajectory setpoint for the vehicle to follow
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [0.0, 0.0, -3.0]  # x, y, z in NED frame
        msg.yaw = 0.0  # Desired yaw angle
        self.trajectory_setpoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    uav_control = UAVControl()
    
    try:
        rclpy.spin(uav_control)
    except KeyboardInterrupt:
        uav_control.get_logger().info('Keyboard interrupt detected, shutting down')
    finally:
        # Clean up
        uav_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 