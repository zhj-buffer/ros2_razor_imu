#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
import serial
import string
import math
import sys
from time import sleep
from lib.serial_commands import *
import yaml
import time

# from time import time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from transforms3d.euler import euler2quat as quaternion_from_euler
from razor_imu_9dof.cfg import imu_config
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi / 180.0
imu_yaw_calibration = 0.0


class RazorImuDriver(Node):
    def __init__(self):
        super().__init__('razor_imu')
        # We only care about the most recent measurement, i.e. queue_size=1
        pub_imu = self.create_publisher(Imu, 'imu', queue_size=1)

        diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', queue_size=1)
        diag_pub_time = self.now()

        imu_msg = Imu()
        imu_msg.orientation_covariance = self.declare_parameter('orientation_covariance').value
        imu_msg.angular_velocity_covariance = self.declare_parameter('velocity_covariance').value
        imu_msg.linear_acceleration_covariance = self.declare_parameter(
            'acceleration_covariance').value
        imu_msg.header.frame_id = self.declare_parameter('frame_header', 'base_imu_link').value

        publish_magnetometer = self.declare_parameter('publish_magnetometer', False).value

        if publish_magnetometer:
            pub_mag = self.create_publisher(MagneticField, 'mag', queue_size=1)
            mag_msg = MagneticField()
            mag_msg.magnetic_field_covariance = self.declare_parameter(
                'magnetic_field_covariance').value
            mag_msg.header.frame_id = self.declare_parameter('frame_header', 'base_imu_link').value
            # should a separate diagnostic for the Magnetometer be done?

        port = self.declare_parameter('port', '/dev/ttyUSB0').value

        # read calibration parameters
        self.calib_dict = {}

        # accelerometer
        self.calib_dict["accel_x_min"] = self.declare_parameter('accel_x_min', -250.0).value
        self.calib_dict["accel_x_max"] = self.declare_parameter('accel_x_max', 250.0).value
        self.calib_dict["accel_y_min"] = self.declare_parameter('accel_y_min', -250.0).value
        self.calib_dict["accel_y_max"] = self.declare_parameter('accel_y_max', 250.0).value
        self.calib_dict["accel_z_min"] = self.declare_parameter('accel_z_min', -250.0).value
        self.calib_dict["accel_z_max"] = self.declare_parameter('accel_z_max', 250.0).value

        # magnetometer
        self.calib_dict["magn_x_min"] = self.declare_parameter('magn_x_min', -600.0).value
        self.calib_dict["magn_x_max"] = self.declare_parameter('magn_x_max', 600.0).value
        self.calib_dict["magn_y_min"] = self.declare_parameter('magn_y_min', -600.0).value
        self.calib_dict["magn_y_max"] = self.declare_parameter('magn_y_max', 600.0).value
        self.calib_dict["magn_z_min"] = self.declare_parameter('magn_z_min', -600.0).value
        self.calib_dict["magn_z_max"] = self.declare_parameter('magn_z_max', 600.0).value
        self.calib_dict["magn_use_extended"] = self.declare_parameter(
            'calibration_magn_use_extended', False).value
        self.calib_dict["magn_ellipsoid_center"] = self.declare_parameter('magn_ellipsoid_center',
                                                                          [0, 0, 0]).value
        self.calib_dict["magn_ellipsoid_transform"] = self.declare_parameter(
            'magn_ellipsoid_transform',
            [[0, 0, 0], [0, 0, 0], [0, 0, 0]]).value

        # gyroscope
        self.calib_dict["gyro_average_offset_x"] = self.declare_parameter('gyro_average_offset_x',
                                                                          0.0).value
        self.calib_dict["gyro_average_offset_y"] = self.declare_parameter('gyro_average_offset_y',
                                                                          0.0).value
        self.calib_dict["gyro_average_offset_z"] = self.declare_parameter('gyro_average_offset_z',
                                                                          0.0).value

        imu_yaw_calibration = self.declare_parameter('imu_yaw_calibration', 0.0).value

        # Check your COM port and baud rate
        self.get_logger().info("Razor IMU -> Opening %s...", port)
        connection_attempts = 5
        for connection_tries in range(0, connection_attempts + 1):
            try:
                ser = serial.Serial(port=port, baudrate=57600, timeout=1)
                break
            except serial.serialutil.SerialException:
                rclpy.get_logger().error(
                    "Razor IMU not found at port " + port + ". Did you specify the correct port in the launch file? Trying %d more times...",
                    5 - connection_tries)

            if connection_tries == connection_attempts:
                # exit
                sys.exit(0)
            time.sleep(3)

        roll = 0
        pitch = 0
        yaw = 0
        seq = 0
        accel_factor = 9.806 / 256.0  # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

        ### configure board ###
        # stop datastream
        self.send_command(ser, STOP_DATASTREAM)
        self.write_and_check_config(ser, self.calib_dict)

        if publish_magnetometer:
            self.send_command(ser, SET_TEXT_EXTENDED_FORMAT_WITH_MAG)
            line_start = LINE_START_WITH_MAG
        else:
            self.send_command(ser, SET_TEXT_EXTENDED_FORMAT_NO_MAG)
            line_start = LINE_START_NO_MAG

        self.send_command(ser, START_DATASTREAM)

        self.get_logger().info("Razor IMU up and running")

        while rclpy.ok():
            line = ser.readline()
            if not line.startswith(line_start):
                self.get_logger().error(1,
                                        "Did not find correct line start in the received IMU message")
                continue
            line = line.replace(line_start, "")  # Delete "#YPRAG=" or "#YPRAGM="
            words = string.split(line, ",")  # Fields split
            if len(words) > 2:
                # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
                yaw_deg = -float(words[0])
                yaw_deg = yaw_deg + imu_yaw_calibration
                if yaw_deg > 180.0:
                    yaw_deg = yaw_deg - 360.0
                if yaw_deg < -180.0:
                    yaw_deg = yaw_deg + 360.0
                yaw = yaw_deg * degrees2rad
                # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
                pitch = -float(words[1]) * degrees2rad
                roll = float(words[2]) * degrees2rad

                # Publish message
                # AHRS firmware accelerations are negated
                # This means y and z are correct for ROS, but x needs reversing
                imu_msg.linear_acceleration.x = -float(words[3]) * accel_factor
                imu_msg.linear_acceleration.y = float(words[4]) * accel_factor
                imu_msg.linear_acceleration.z = float(words[5]) * accel_factor

                imu_msg.angular_velocity.x = float(words[6])
                # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
                imu_msg.angular_velocity.y = -float(words[7])
                # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
                imu_msg.angular_velocity.z = -float(words[8])

                if publish_magnetometer:
                    # according to line 178 the units published of the mag are mGauss
                    # REP103 specifys the units of magnetic field to be Teslas
                    # The axis of the MPU magnetometer are X forward, Y right and Z down
                    #  when the chip is facing forward, in the sparkfun board, the chip is facing the left side
                    # but Sparkfun the firmware interchanges x and y and changes the sign of y
                    # so to get it to REP103 we need to make X and Z negative
                    # The magn frames form sparkfun can be seen in line 178 from Sensors.ino
                    mag_msg.magnetic_field.x = float(words[9]) * 1e-7
                    mag_msg.magnetic_field.y = -float(words[10]) * 1e-7
                    mag_msg.magnetic_field.z = -float(words[11]) * 1e-7
                    # check frame orientation and units

            q = quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.header.stamp = self.now()
            imu_msg.header.seq = seq
            seq = seq + 1
            pub_imu.publish(imu_msg)

            if publish_magnetometer:
                mag_msg.header.stamp = imu_msg.header.stamp
                mag_msg.header.seq = imu_msg.header.seq
                pub_mag.publish(mag_msg)

            if (diag_pub_time < rclpy.get_time()):
                diag_pub_time += 1
                diag_arr = DiagnosticArray()
                diag_arr.header.stamp = rclpy.get_rostime()
                diag_arr.header.frame_id = '1'
                diag_msg = DiagnosticStatus()
                diag_msg.name = 'Razor_Imu'
                diag_msg.level = DiagnosticStatus.OK
                diag_msg.message = 'Received AHRS measurement'
                diag_msg.values.append(KeyValue('roll (deg)',
                                                str(roll * (180.0 / math.pi))))
                diag_msg.values.append(KeyValue('pitch (deg)',
                                                str(pitch * (180.0 / math.pi))))
                diag_msg.values.append(KeyValue('yaw (deg)',
                                                str(yaw * (180.0 / math.pi))))
                diag_msg.values.append(KeyValue('sequence number', str(seq)))
                diag_arr.status.append(diag_msg)
                diag_pub.publish(diag_arr)

            self.spin_some()
        ser.close

    def send_command(self, serial_instance, command, value=None):
        if value is None:
            cmd = command + chr(13)
        else:
            cmd = command + str(value) + chr(13)
        self.get_logger().info("Razor IMU -> Sending: %s", cmd)
        expected_len = len(cmd)
        res = serial_instance.write(cmd)
        if (res != expected_len):
            self.get_logger().error(
                "Razor IMU -> Expected serial command len (%d) didn't match amount of bytes written (%d) for command %s",
                expected_len, res, command)
        sleep(0.05)  # Don't spam serial with too many commands at once

    def write_and_check_config(self, serial_instance, calib_dict):
        self.send_command(serial_instance, SET_CALIB_ACC_X_MIN, calib_dict["accel_x_min"])
        self.send_command(serial_instance, SET_CALIB_ACC_X_MAX, calib_dict["accel_x_max"])
        self.send_command(serial_instance, SET_CALIB_ACC_Y_MIN, calib_dict["accel_y_min"])
        self.send_command(serial_instance, SET_CALIB_ACC_Y_MAX, calib_dict["accel_y_max"])
        self.send_command(serial_instance, SET_CALIB_ACC_Z_MIN, calib_dict["accel_z_min"])
        self.send_command(serial_instance, SET_CALIB_ACC_Z_MAX, calib_dict["accel_z_max"])

        if not calib_dict["magn_use_extended"]:
            self.send_command(serial_instance, SET_MAG_X_MIN, calib_dict["magn_x_min"])
            self.send_command(serial_instance, SET_MAG_X_MAX, calib_dict["magn_x_max"])
            self.send_command(serial_instance, SET_MAG_Y_MIN, calib_dict["magn_y_min"])
            self.send_command(serial_instance, SET_MAG_Y_MAX, calib_dict["magn_y_max"])
            self.send_command(serial_instance, SET_MAG_Z_MIN, calib_dict["magn_z_min"])
            self.send_command(serial_instance, SET_MAG_Z_MAX, calib_dict["magn_z_max"])
        else:
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_CENTER_0,
                              self.calib_dict["magn_ellipsoid_center"][0])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_CENTER_1,
                              self.calib_dict["magn_ellipsoid_center"][1])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_CENTER_2,
                              self.calib_dict["magn_ellipsoid_center"][2])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_0_0,
                              self.calib_dict["magn_ellipsoid_transform"][0][0])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_0_1,
                              self.calib_dict["magn_ellipsoid_transform"][0][1])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_0_2,
                              self.calib_dict["magn_ellipsoid_transform"][0][2])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_1_0,
                              self.calib_dict["magn_ellipsoid_transform"][1][0])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_1_1,
                              self.calib_dict["magn_ellipsoid_transform"][1][1])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_1_2,
                              self.calib_dict["magn_ellipsoid_transform"][1][2])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_2_0,
                              self.calib_dict["magn_ellipsoid_transform"][2][0])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_2_1,
                              self.calib_dict["magn_ellipsoid_transform"][2][1])
            self.send_command(serial_instance, SET_MAG_ELLIPSOID_TRANSFORM_2_2,
                              self.calib_dict["magn_ellipsoid_transform"][2][2])

        self.send_command(serial_instance, SET_GYRO_AVERAGE_OFFSET_X,
                          self.calib_dict["gyro_average_offset_x"])
        self.send_command(serial_instance, SET_GYRO_AVERAGE_OFFSET_Y,
                          self.calib_dict["gyro_average_offset_y"])
        self.send_command(serial_instance, SET_GYRO_AVERAGE_OFFSET_Z,
                          self.calib_dict["gyro_average_offset_z"])

        self.send_command(serial_instance, GET_CALIBRATION_VALUES)
        config = ""
        for _ in range(0, 21):
            # Format each line received from serial into proper yaml with lowercase variable names
            config += serial_instance.readline().lower().replace(":", ": ")
            # TODO: round the numbers, otherwise we will get false negatives in the check phase

        config_parsed = yaml.load(config)
        for key in self.calib_dict:
            if config_parsed[key] != self.calib_dict[key]:
                self.get_logger().warning(
                    "The calibration value of [%s] did not match. Expected: %s, received: %s",
                    key, str(self.calib_dict[key]), str(config_parsed[key]))


def main(args=None):
    rclpy.init(args=args)
    node = RazorImuDriver()

    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
