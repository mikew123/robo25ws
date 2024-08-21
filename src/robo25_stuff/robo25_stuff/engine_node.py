import rclpy
import json
import serial
import time

from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class EngineNode(Node):
    '''
    This processes the serial port from the RP2040 in the
    engine compartment
    '''

    timerRateHz = 220.0; # Rate to check serial port for messages

    # Try opening serial ports and checking "id"
    serialPorts = ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3")
    serialId = "engine"

    # Dont know how to enable by-id on the Docker container
    #serial_port:str = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625887D3477130-if00"

    def __init__(self):
        super().__init__('engine_node')

        self.engine_serial_port = self.determineSerialPort(self.serialId)
        if self.engine_serial_port == None :
            self.get_logger().info(f"ImuGpsNode Serial port id {self.serialId} not found - Exit node")
            exit(1)
        
        self.engine_systat_publisher = self.create_publisher(String, 'engine_systat', 10)
        self.engine_rx_publisher = self.create_publisher(String, 'engine_rx', 10)

        self.robo25_json_subscription = self.create_subscription(String, 'robo25_json', self.robo25_json_callback, 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)
        
        # configure interface
        self.engine_serial_port.write("{\"cfg\":{\"rxe\":true, \"sse\":true, \"fsa\":\"ena\"}}".encode())
        # # DEBUG loopback Enable receiver RX data
        # self.engine_serial_port.write("{\"cfg\":{\"rxe\":true, \"fsa\":\"dis\"}}".encode())
        # self.engine_rx_subscription = self.create_subscription(String, 'engine_rx', self.engine_rx_callback, 10)
        
        self.get_logger().info(f"EngineNode Started")

    # check serial ports to find the one that matches the id
    def determineSerialPort(self, id) :
        for serial_port in self.serialPorts :
            try:
                self.get_logger().info(f"Try {serial_port}")
                node_serial_port = serial.Serial(serial_port, 1000000, timeout=0.1)
                node_serial_port.write("{\"id\":0}".encode())
                time.sleep(1)
                # read port multiple times checking for "id"
                for i in range(10):
                    received_data = node_serial_port.readline().decode().strip()
                    self.get_logger().info(f"Received engine json: {received_data}")
                    try :
                        packet = json.loads(received_data)
                        if "id" in packet:
                            if packet.get("id") == id :
                                self.get_logger().info(f"Found {id} serial port: {serial_port}")
                                return node_serial_port
                    except Exception as ex:
                        self.get_logger().info(f"Bad json.loads packet {received_data}: {ex}")
            except Exception as ex:
                self.get_logger().info(f"Serial port {serial_port} did not open: {ex}")
        return None

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.engine_serial_port.in_waiting > 0:
            try :
                received_data = self.engine_serial_port.readline().decode().strip()
                #self.get_logger().info(f"Received engine json: {received_data}")
            except Exception as ex:
                self.get_logger().error(f"Engine serial read failure : {ex}")
                return
            
            try :
                packet = json.loads(received_data)
                if "systat" in packet :
                    systat = json.dumps(packet.get("systat"))
                    msg = String()
                    msg.data = systat
                    self.engine_systat_publisher.publish(msg)
                elif "rx" in packet :
                    rx = json.dumps(packet.get("rx"))
                    msg = String()
                    msg.data = rx
                    self.engine_rx_publisher.publish(msg)
                    
                    # #debug loopback
                    # drv = packet.get("rx")
                    # drv_str = "{\"drv\":"+json.dumps(packet.get("rx"))+"}"
                    # self.engine_serial_port.write((drv_str+"\n").encode())
                    # self.get_logger().info(f"{drv_str=}")  
                else :
                    self.get_logger().info(f"Engine serial json unknown tag : {received_data}")
                    return  
            except Exception as ex:
                self.get_logger().error(f"Engine serial json failure {ex} : {received_data}")
                return

    # # Loopback test
    # def engine_rx_callback(self, msg:String) -> None :
    #     packet_bytes = msg.data
    #     packet = json.loads(packet_bytes)
    #     drvCmd = {"drv": packet}
    #     drvStr = json.dumps(drvCmd)
    #     self.engine_serial_port.write((drvStr+"\n").encode())
    #     self.get_logger().info(f"{drvStr=}")  

    # modes encoded as JSON strings
    def robo25_json_callback(self, msg:String) -> None :
        # send nav modes to watch serial
        packet_bytes = msg.data

        try :
            packet = json.loads(packet_bytes)
            if "engine_cfg" in packet :
                if self.nav_modeReady == False :
                    self.nav_mode = packet_bytes
                    self.nav_modeReady = True
                
        except Exception as ex:
            self.get_logger().error(f"watch serial robo25_json_callback exception {ex}")

def main(args=None):
    rclpy.init(args=args)

    node = EngineNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
