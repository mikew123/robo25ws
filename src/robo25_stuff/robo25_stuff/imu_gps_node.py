import rclpy
import json
import serial

from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu

class ImuGpsNode(Node):
    '''
    This processes the serial port from the RP2040 for the IMU and GPS sensors
    in the cabin
    '''

    timerRateHz = 220.0; # Rate to check serial port for messages

    # Hane not figured out how to get bu-id in Docker yet
    serial_port = "/dev/ttyACM1"

    # Dont know how to enable by-id on the Docker container
    #serial_port:str = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625887D3477130-if00"

    laccJsonPacket = None
    rvelJsonPacket = None
    rvecJsonPacket = None

    def __init__(self):
        super().__init__('imu_gps_node')

        self.imu_gps_serial_port = serial.Serial(self.serial_port, 1000000)
        
        self.imu_test_publisher = self.create_publisher(String, 'imu_test', 10)
        self.imu_msg_publisher = self.create_publisher(Imu, 'imu_msg', 10)


        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)
        
        # configure interface
        #self.engine_serial_port.write("{\"cfg\":{\"rxe\":false, \"fsa\":\"ena\"}}".encode())
        
        self.get_logger().info(f"ImuGpsNode Started")

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.imu_gps_serial_port.in_waiting > 0:
            try :
                received_data = self.imu_gps_serial_port.readline().decode().strip()
#                self.get_logger().info(f"Received engine json: {received_data}")
            except Exception as ex:
                self.get_logger().error(f"IMU GPS serial read failure : {ex}")
                return

            try :
                packet = json.loads(received_data)
                if "imu" in packet :
                    self.imuPublish(packet.get("imu"))
                else :
                    self.get_logger().info(f"Engine serial json unknown : {received_data}")
                    return  
            except Exception as ex:
                self.get_logger().error(f"Engine serial json failure {ex} : {received_data}")
                return

    def imuPublish(self, imuJsonPacket) :        
        # TODO: collect lacc, rvel, rvec packets with same seq# and publish imu message
        try :
            if "lacc" in imuJsonPacket :
                self.laccJsonPacket = imuJsonPacket.get("lacc")
            elif "rvel" in imuJsonPacket :
                self.rvelJsonPacket = imuJsonPacket.get("rvel")
            elif "rvec" in imuJsonPacket :
                self.rvecJsonPacket = imuJsonPacket.get("rvec")
            else :
                self.get_logger().error("")
        except Exception as ex:
            self.get_logger().error(f"IMU json unexpected {ex} : {json.dumps(imuJsonPacket)}")
            return

        if self.laccJsonPacket!=None and  self.rvelJsonPacket!=None and  self.rvecJsonPacket!=None :
            laccSeq = self.laccJsonPacket.get("seq")
            rvelSeq = self.rvelJsonPacket.get("seq")
            rvecSeq = self.rvecJsonPacket.get("seq")
            if laccSeq==rvelSeq and rvelSeq==rvecSeq :
                self.get_logger().info(f"{laccSeq=} {rvelSeq=} {rvecSeq=}")
                msg = Imu()
                self.imu_msg_publisher.publish(msg)
            

        imuJsonStr = json.dumps(imuJsonPacket)
        msg = String()
        msg.data = imuJsonStr
        self.imu_test_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = ImuGpsNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
