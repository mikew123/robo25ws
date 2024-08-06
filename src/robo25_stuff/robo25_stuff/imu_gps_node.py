import rclpy
import json
import serial

from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class ImuGpsNode(Node):
    '''
    This processes the serial port from the RP2040 for the IMU and GPS sensors
    in the cabin
    '''

    timerRateHz = 110.0; # Rate to check serial port for messages

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
        self.gps_msg_publisher = self.create_publisher(NavSatFix, 'gps_msg', 10)


        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)
        
        # configure interface
        self.imu_gps_serial_port.write("{\"cfg\":{\"imu\":true, \"gps\":true}}".encode())
        
        self.get_logger().info(f"ImuGpsNode Started")

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.imu_gps_serial_port.in_waiting > 0:
            try :
                received_data = self.imu_gps_serial_port.readline().decode().strip()
                #self.get_logger().info(f"Received engine json: {received_data}")
            except Exception as ex:
                self.get_logger().error(f"IMU GPS serial read failure : {ex}")
                return

            try :
                unknown = True
                packet = json.loads(received_data)
                if "imu" in packet :
                    self.imuPublish(packet.get("imu"))
                    unknown = False
                if "gps" in packet :
                    self.gpsPublish(packet.get("gps"))
                    unknown = False
                if unknown :
                    self.get_logger().info(f"IMU GPS serial json unknown : {received_data}")
                    return  
            except Exception as ex:
                self.get_logger().error(f"IMU GPS serial json failure {ex} : {received_data}")
                return
            
    def gpsPublish(self, gpsJsonPacket) :        
        #self.get_logger().info(f"gpsPublish : {gpsJsonPacket=}")

        msg = NavSatFix();

        # NOTE: should we use the imu timestamp to get better accuracy?
        msg.header.stamp = self.get_clock().now().to_msg()
        # The IMU is located close to the rear differential 
        # and aligned XY so no offest is needed
        msg.header.frame_id = "base_link"

        # TODO: status fields

        msg.latitude  = 1e-7*gpsJsonPacket.get("lat")
        msg.longitude = 1e-7*gpsJsonPacket.get("lon")
        msg.altitude  = 1e-3*gpsJsonPacket.get("alt") # mm to Meters

        self.gps_msg_publisher.publish(msg)

    def imuPublish(self, imuJsonPacket) :        
        #self.get_logger().info(f"imuPublish : {imuJsonPacket=}")

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

            if self.laccJsonPacket!=None and  self.rvelJsonPacket!=None and  self.rvecJsonPacket!=None :
                laccSeq = self.laccJsonPacket.get("seq")
                rvelSeq = self.rvelJsonPacket.get("seq")
                rvecSeq = self.rvecJsonPacket.get("seq")
                if laccSeq==rvelSeq and rvelSeq==rvecSeq :
                    #self.get_logger().info(f"{laccSeq=} {rvelSeq=} {rvecSeq=}")
                    msg = Imu()

                    # NOTE: should we use the imu timestamp to get better accuracy?
                    msg.header.stamp = self.get_clock().now().to_msg()
                    # The IMU is located at the centroid of the rear differential 
                    # and aligned XY so no offest is needed
                    msg.header.frame_id = "base_link"

                    #self.get_logger().info(f"imuPublish : {self.rvecJsonPacket=}")
                    msg.orientation.x = float(self.rvecJsonPacket.get("i"))
                    msg.orientation.y = float(self.rvecJsonPacket.get("j"))
                    msg.orientation.z = float(self.rvecJsonPacket.get("k"))
                    msg.orientation.w = float(self.rvecJsonPacket.get("real"))

                    #self.get_logger().info(f"imuPublish : {self.rvelJsonPacket=}")
                    msg.angular_velocity.x =  float(self.rvelJsonPacket.get("x"))
                    msg.angular_velocity.y =  float(self.rvelJsonPacket.get("y"))
                    msg.angular_velocity.z =  float(self.rvelJsonPacket.get("z"))

                    #self.get_logger().info(f"imuPublish : {self.laccJsonPacket=}")
                    msg.linear_acceleration.x = float(self.laccJsonPacket.get("x"))
                    msg.linear_acceleration.y = float(self.laccJsonPacket.get("y"))
                    msg.linear_acceleration.z = float(self.laccJsonPacket.get("z"))

                    self.imu_msg_publisher.publish(msg)

        except Exception as ex:
            self.get_logger().error(f"imuPublish json exception : {ex}")
            return


        # imuJsonStr = json.dumps(imuJsonPacket)
        # msg = String()
        # msg.data = imuJsonStr
        # self.imu_test_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = ImuGpsNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
