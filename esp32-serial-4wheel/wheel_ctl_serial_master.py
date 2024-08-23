

#ROS2 publish(float64multiarray): /C620/actual_rad "speed1, speed2, speed3, speed4"
#ROS2 subscribe1(float64multiarray): /rover/left_targets "speed1, speed2"
#ROS2 subscribe2(float64multiarray): /rover/right_targets "speed3, speed4" 

#Serial send: motor wheel rad "speed1, speed2, speed3, speed4"
#Serial receive: motor actual rad "speed1, speed2, speed3, speed4"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

class Serial4Wheel(Node):
    def __init__(self):
        super().__init__('serial_4wheel')
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/C620/actual_rad', 10)
        self.wheel_sub = self.create_subscription(Float64MultiArray, '/rover/targets', self.wheel_cb, 10)
        #self.timer = self.create_timer(0.1, self.send_serial)
        #self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.ser = serial.Serial('/dev/ttyACM1', 115200)
        self.get_logger().info('Serial 4 wheel node has started')
        self.actual_wheel_data = Float64MultiArray()
        self.serial_wheel_data=[0,0,0,0]


    def wheel_cb(self, msg):
        self.serial_wheel_data[0] = msg.data[0]
        self.serial_wheel_data[1] = msg.data[1]
        self.serial_wheel_data[2] = msg.data[2]
        self.serial_wheel_data[3] = msg.data[3]
        self.send_serial()
        
        

        
    def send_serial(self):
        self.ser.write(f'{self.serial_wheel_data[0]}, {self.serial_wheel_data[1]}, {self.serial_wheel_data[2]}, {self.serial_wheel_data[3]}'.encode())

    #パブリッシャのループ　TODO:ほかのシリアルインターフェースを用いる
    # def run(self):
    #     while True:
    #         try:
    #             data = self.ser.readline().decode().strip()
    #             data = data.split(',')
    #             if len(data) == 4:
    #                 self.actual_wheel_data.data = [float(data[0]), float(data[1]), float(data[2]), float(data[3])]
    #                 self.wheel_pub.publish(self.actual_wheel_data)
    #         except Exception as e:
    #             self.get_logger().error(str(e))
    #             break

    def stop(self):
        self.ser.close()
        self.get_logger().info('Serial 4 wheel node has stopped')

def main(args=None):
    rclpy.init(args=args)
    serial_4wheel = Serial4Wheel()
    serial_4wheel.run()
    rclpy.spin(serial_4wheel)
    serial_4wheel.stop()

if __name__ == '__main__':
    main()



