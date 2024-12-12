#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime
import serial
import re
from nr5g_quectel_at.msg import ATQuectelNR5G  # Custom message
from std_msgs.msg import String


class NR5GQuectelPublisher(Node):
    def __init__(self):
        super().__init__('nr5g_quectel_publisher')

        # Serial port configuration
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1
        )

        # Publisher setup
        self.publisher_ = self.create_publisher(ATQuectelNR5G, 'quectel_nr5g', 10)

        # Timer to call the publish method at the desired interval
        self.refresh = 0.25 # 200 ms between each msg
        self.timer = self.create_timer(self.refresh, self.publish_status)

        self.get_logger().info('Node has been initialized')

    def publish_status(self):
        measurements = self.parse_nr5g()
        print(measurements)

        if not measurements:
            self.get_logger().warning('Failed to parse measurements.')
            return

        nr5g_params = measurements["net_param"]
        msg = ATQuectelNR5G()

        # Assign values to the custom message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.net_type = nr5g_params.get("net_type", "N/A")
        msg.rsrp = self.safe_float_conversion(nr5g_params.get("rsrp"))
        msg.rsrq = self.safe_float_conversion(nr5g_params.get("rsrq"))
        msg.sinr = self.safe_float_conversion(nr5g_params.get("sinr"))
        msg.band = str(nr5g_params.get("band", "N/A"))
        msg.bw_dl = self.safe_int_conversion(nr5g_params.get("bandwidth_dl"))
        msg.cell_id = self.safe_int_conversion(nr5g_params.get("cell_id"))
        msg.gnb = self.safe_int_conversion(nr5g_params.get("gNB"))
        msg.sector_id = self.safe_int_conversion(nr5g_params.get("sector_ID"))
        msg.rssi_0 = self.safe_float_conversion(nr5g_params.get("rssi_0"))
        msg.rssi_1 = self.safe_float_conversion(nr5g_params.get("rssi_1"))
        msg.rssi_2 = self.safe_float_conversion(nr5g_params.get("rssi_2"))
        msg.rssi_3 = self.safe_float_conversion(nr5g_params.get("rssi_3"))

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Published 5G network status')

    def parse_nr5g(self):
        # Initialize variables
        nr5g_rssi = [None] * 4
        nr5g_rsrq = [None] * 4
        nr5g_rsrp = [None] * 4
        nr5g_sinr = [None] * 4
        nr5g_rsrp_m, nr5g_rsrq_m, nr5g_sinr_m, nr5g_band, nr5g_bw_dl, nr5g_bw_ul = [None] * 6
        nr5g_cell_id, gNodeB_ID, nr5g_cell_id, sector_ID = [None] * 4
        gNodeB_ID_Length = 28
        # Define a mapping for DL bandwidth
        bandwidth_mapping = {0: "5 MHz", 1: "10 MHz", 2: "15 MHz", 3: "20 MHz", 4: "25 MHz", 5: "30 MHz", 6: "40 MHz", 7: "50 MHz", 8: "60 MHz", 9: "70 MHz", 10: "80 MHz", 11: "90 MHz", 12: "100 MHz", 13: "200 MHz", 14: "400 MHz"}

        # Send command to the device
        self.ser.write(b'at+qrssi?\r\n')
        while True:
            line = self.ser.readline().replace(b'\r\n', b'').decode('utf-8')
            print(f"DEBUG: {line}")  # Debug print to trace each line #+QRSSI: -87,-80,-85,-77,NR5G
            if '+QRSSI:' in line:
                parts = line.replace("+QRSSI:", "").strip().split(',')
                nr5g_rssi = [float(part) for part in parts[:4]]

            if line.startswith('OK'):
                break

        self.ser.write(b'at+qeng="servingcell"\r\n')
        while True:
            line = self.ser.readline().replace(b'\r\n', b'').decode('utf-8')
            print(f"DEBUG: {line}")  # Debug print to trace each line +QENG: "servingcell","NOCONN","NR5G-SA","TDD",240,68,FAD5383F2,310,64,634080,78,12,-88,-10,23,1,-
            if '+QENG:' in line:
                parts = line.replace("+QENG:", "").strip().split(',')
            
                nr5g_cell_id = int(parts[6], 16)
                gNodeB_ID = nr5g_cell_id//(2 ** (36 - gNodeB_ID_Length))
                sector_ID = nr5g_cell_id - (gNodeB_ID * (2 ** (36 - gNodeB_ID_Length)))
                nr5g_band = "Band "+str(parts[10])
                nr5g_bw_dl = bandwidth_mapping.get(int(parts[11]))
                nr5g_rsrp_m = float(parts[12])
                nr5g_rsrq_m = float(parts[13])
                nr5g_sinr_m = float(parts[14])

            if line.startswith('OK'):
                break

        # Return the parsed dictionary
        return {
            "timestamp": str(datetime.now()),
            "net_param": {
                "net_type": "5G",
                "rsrp": nr5g_rsrp,
                "rsrq": nr5g_rsrq,
                "sinr": nr5g_sinr,
                "band": nr5g_band,
                "bandwidth_dl": nr5g_bw_dl,
                "rssi_0": nr5g_rssi[0],
                "rssi_1": nr5g_rssi[1],
                "rssi_2": nr5g_rssi[2],
                "rssi_3": nr5g_rssi[3],
                "cell_id": nr5g_cell_id,
                "gNB": gNodeB_ID,
                "sector_ID": sector_ID,
            }
        }

    @staticmethod
    def safe_float_conversion(value):
        return float(value) if value not in [None, '---'] else float('nan')

    @staticmethod
    def safe_int_conversion(value):
        return int(value) if value not in [None, '---'] else 0

def main(args=None):
    rclpy.init(args=args)
    node = NR5GQuectelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
