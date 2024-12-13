import rospy
import serial
import time
from datetime import datetime
import re
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from nr5g_quectel_at.msg import at_quectel_nr5g

# Read interval
refresh = 0.25

gNodeB_ID_Length = 28

# Configuration of the serial port comm
ser = serial.Serial(timeout=1)
ser.baudrate = 115200
ser.port = '/dev/ttyUSB2'
ser.open()

def parse_nr5g():
    # Initialize variables
    nr5g_rssi = [None] * 4
    nr5g_rsrp, nr5g_rsrq, nr5g_sinr, nr5g_band, nr5g_bw_dl = [None] * 5
    nr5g_cell_id, gNodeB_ID, nr5g_cell_id, sector_ID = [None] * 4
    gNodeB_ID_Length = 28
    # Define a mapping for DL bandwidth
    bandwidth_mapping = {0: "5", 1: "10", 2: "15", 3: "20", 4: "25", 5: "30", 6: "40", 7: "50", 8: "60", 9: "70", 10: "80", 11: "90", 12: "100", 13: "200", 14: "400"}

    # Send command to the device
    ser.write(b'at+qrssi?\r\n')
    while True:
        line = ser.readline().replace(b'\r\n', b'').decode('utf-8')
        print(f"DEBUG: {line}")  # Debug print to trace each line #+QRSSI: -87,-80,-85,-77,NR5G
        if '+QRSSI:' in line:
            parts = line.replace("+QRSSI:", "").strip().split(',')
            nr5g_rssi = [float(part) for part in parts[:4]]

        if line.startswith('OK'):
            break

    ser.write(b'at+qeng="servingcell"\r\n')
    while True:
        line = ser.readline().replace(b'\r\n', b'').decode('utf-8')
        print(f"DEBUG: {line}")  # Debug print to trace each line +QENG: "servingcell","NOCONN","NR5G-SA","TDD",240,68,FAD5383F2,310,64,634080,78,12,-88,-10,23,1,-
        if '+QENG:' in line:
            parts = line.replace("+QENG:", "").strip().split(',')
        
            nr5g_cell_id = int(parts[6], 16)
            gNodeB_ID = nr5g_cell_id//(2 ** (36 - gNodeB_ID_Length))
            sector_ID = nr5g_cell_id - (gNodeB_ID * (2 ** (36 - gNodeB_ID_Length)))
            nr5g_band = "Band "+str(parts[10])
            nr5g_bw_dl = bandwidth_mapping.get(int(parts[11]))
            nr5g_rsrp = float(parts[12])
            nr5g_rsrq = float(parts[13])
            nr5g_sinr = float(parts[14])


        if line.startswith('OK'):
            break

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

def safe_float_conversion(value):
    return float(value) if value not in [None, '---'] else float('nan')


def safe_int_conversion(value):
    return int(value) if value not in [None, '---'] else 0

def scan():
    nr5g_data = parse_nr5g()
    return {"nr5g": nr5g_data}

def publish_5g_status():
    rospy.init_node('nr5g_quectel_at', anonymous=True)
    pub = rospy.Publisher('quectel_nr5g', at_quectel_nr5g, queue_size=10)
    rate = rospy.Rate(1/refresh)  # Frequency based on refresh rate

    while not rospy.is_shutdown():
        measurements = scan()
        nr5g_params = measurements["nr5g"]["net_param"]
        msg = at_quectel_nr5g()
        msg.header.stamp = rospy.Time.now()
        msg.net_type = nr5g_params["net_type"]
        msg.rsrp = safe_float_conversion(nr5g_params["rsrp"])
        msg.rsrq = safe_float_conversion(nr5g_params["rsrq"])
        msg.sinr = safe_float_conversion(nr5g_params["sinr"])
        msg.band = nr5g_params["band"]
        msg.bw_dl = safe_int_conversion(nr5g_params["bandwidth_dl"])
        msg.cell_id = safe_int_conversion(nr5g_params["cell_id"])
        msg.gNB = safe_int_conversion(nr5g_params["gNB"])
        msg.sector_ID = safe_int_conversion(nr5g_params["sector_ID"])
        msg.rssi_0 = safe_float_conversion(nr5g_params["rssi_0"])
        msg.rssi_1 = safe_float_conversion(nr5g_params["rssi_1"])
        msg.rssi_2 = safe_float_conversion(nr5g_params["rssi_2"])
        msg.rssi_3 = safe_float_conversion(nr5g_params["rssi_3"])

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_5g_status()
    except rospy.ROSInterruptException:
        pass
