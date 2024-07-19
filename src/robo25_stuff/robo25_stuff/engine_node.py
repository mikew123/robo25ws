import rclpy
import json
import serial

from rclpy.node import Node
from std_msgs.msg import String

class EngineNode(Node):
    '''
    This processes the serial port from the RP2040 in the
    engine compartment
    '''

    timerRateHz = 1.0; # Rate to check serial port for messages

    serial_port = "/dev/ttyACM0"

    # Dont know how to enable by-id on the Docker container
    #serial_port:str = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625887D3477130-if00"

    def __init__(self):
        super().__init__('engine_node')

        self.sensor_serial_port = serial.Serial(self.serial_port, 1000000)
        # configure interface
        #self.sensor_serial_port.write(f"MODE ROS2\n".encode()) # extra tfor startup


        self.engine_msg_publisher = self.create_publisher(String, 'engine_msg', 10)

        self.robo25_json_subscription = self.create_subscription(String, 'robo25_json', self.robo24_json_callback, 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        self.get_logger().info(f"EngineNode Started")

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.sensor_serial_port.in_waiting > 0:
            received_data = self.sensor_serial_port.readline().decode().strip()
            self.get_logger().info(f"Received engine json: {received_data}")
            
            packet_bytes = received_data

            try :
                packet = json.loads(packet_bytes)
                #if "str" in packet :
               
                
            except Exception as ex:
                self.get_logger().error(f"watch serial robo24_json_callback exception {ex}")

            str = String()
            str.data = received_data
            #str.data = "Test message to publish"

            self.engine_msg_publisher.publish(str)
            self.get_logger().info(f"{str}")

    # modes encoded as JSON strings
    def robo24_json_callback(self, msg:String) -> None :
        # send nav modes to watch serial
        packet_bytes = msg.data

        try :
            packet = json.loads(packet_bytes)
            if "run_state" in packet :
                if self.nav_modeReady == False :
                    self.nav_mode = packet_bytes
                    self.nav_modeReady = True
                
        except Exception as ex:
            self.get_logger().error(f"watch serial robo24_json_callback exception {ex}")

def main(args=None):
    rclpy.init(args=args)

    node = EngineNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
