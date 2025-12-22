import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int8MultiArray, Int8
from geometry_msgs.msg import Twist

class AutoStopTest(Node):
    def __init__(self):
        super().__init__('auto_stop_test')

        # --- Parameters (รับค่าจาก YAML) ---
        self.declare_parameter('move_speed', 0.6)       # ความเร็วเดินหน้า (m/s)
        self.declare_parameter('stop_delay', 5.0)       # หน่วงเวลาหากไม่มีสิ่งกีดขวางให้เดินหน้าต่อ  (s)
        self.declare_parameter('sensor_timeout', 1.0)   # time out หากไม่ได้รับข้อมูล sensor  (s)
        self.move_speed = self.get_parameter('move_speed').value
        self.stop_delay = self.get_parameter('stop_delay').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value

        # --- Variables ---
        self.test_mode = 0    # 0 = Manual (Joy), 1 = Auto Test

        # เก็บเวลาล่าสุดที่เจอสิ่งกีดขวาง (เริ่มมาให้เซ็ตเป็นปัจจุบัน เพื่อให้รอตรวจสอบก่อนเริ่มเดิน)
        self.last_blocked_time = self.get_clock().now()

        # เก็บเวลาล่าสุดที่ได้รับข้อมูล Zone (เอาไว้เช็ค Watchdog)
        self.last_zone_msg_time = self.get_clock().now()

        # --- Subscribers ---
        # 1. รับค่าโหมด (0 หรือ 1)
        self.create_subscription(Int8, '/test_mode', self.mode_callback, 10)
        
        # 2. รับค่า Array จาก TOF Node
        self.create_subscription(Int8MultiArray, '/tof_zone_array', self.zone_callback, 10)

        # --- Publisher ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer for Watchdog ---
        # เช็คทุกๆ 0.1 วินาที (10Hz)
        self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info('Auto Stop Node Started.')
        self.get_logger().info(f'Config: Speed={self.move_speed}, Delay={self.stop_delay}, Timeout={self.sensor_timeout}')
        self.get_logger().info('Waiting for command: ros2 topic pub /test_mode std_msgs/msg/Int8 "{data: 1}"')

    def mode_callback(self, msg):
        prev_mode = self.test_mode
        self.test_mode = msg.data
        
        if self.test_mode == 1:
            # อัปเดตค่าจาก Param ทุกครั้งที่เข้าโหมด
            self.move_speed = self.get_parameter('move_speed').value
            self.stop_delay = self.get_parameter('stop_delay').value
            self.sensor_timeout = self.get_parameter('sensor_timeout').value

            # รีเซ็ตเวลาเพื่อให้เริ่มนับใหม่ (ป้องกันการพุ่งตัวทันทีที่เข้าโหมด)
            self.last_blocked_time = self.get_clock().now()

            # รีเซ็ตเวลาเพื่อไม่ให้ Timeout ทันทีที่เริ่ม
            self.last_zone_msg_time = self.get_clock().now()

            self.get_logger().info(f'>>> ENTER TEST MODE: Speed {self.move_speed} <<<')
            self.get_logger().info(f'>>> ENTER TEST MODE (Wait {self.stop_delay}s first) <<<')
        elif self.test_mode == 0:
            self.get_logger().info('<<< EXIT TEST MODE: Returned control to Joy >>>')
            # เพื่อความปลอดภัย ส่ง Stop ไปหนึ่งทีก่อนปล่อยมือ
            self.stop_robot() 

    def zone_callback(self, msg):
        # อัปเดตเวลาล่าสุดที่ได้รับข้อมูล
        self.last_zone_msg_time = self.get_clock().now()

        # ถ้าไม่ได้อยู่ในโหมด Test ให้จบฟังก์ชันเลย (ไม่ส่ง cmd_vel)
        if self.test_mode == 0:
            return

        # เช็คว่าข้อมูลมาครบ 5 ตัวไหม
        if len(msg.data) < 5:
            return

        # อัปเดตข้อมูล (Index 2 คือ Center)
        # Array: [OuterL, InnerL, Center, InnerR, OuterR]
        is_center_blocked = (msg.data[2] == 1)
        current_time = self.get_clock().now()

        twist = Twist()

        if is_center_blocked:
            # เจอสิ่งกีดขวางตรงกลาง -> หยุดและรีเซ็ตเวลา
            self.last_blocked_time = current_time
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn('OBSTACLE DETECTED (Center)! Stopping...', throttle_duration_sec=1.0)
        else:
            # ทางสะดวก -> เช็คว่าผ่านไปนานเท่าไหร่แล้ว(หน่วยวินาที) ถ้ายังไม่ครบเวลาหยุดรอ -> ครบเวลาแล้ว -> เดินหน้า
            elapsed_time = (current_time - self.last_blocked_time).nanoseconds / 1e9
            
            if elapsed_time < self.stop_delay:
                # ยังไม่ครบเวลาหน่วง: หยุดรอ
                remaining = self.stop_delay - elapsed_time
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(f'Waiting for safety... {remaining:.1f}s', throttle_duration_sec=1.0)
            else:
                # ครบเวลาแล้ว: เดินหน้า
                twist.linear.x = self.move_speed
                twist.angular.z = 0.0
                # self.get_logger().info('Path Clear & Safe: Moving', throttle_duration_sec=2.0)

        # ส่งคำสั่งขับเคลื่อน
        self.cmd_vel_pub.publish(twist)

    def watchdog_callback(self):
            # ทำงานเฉพาะตอนอยู่ในโหมด Auto Test
            if self.test_mode == 1:
                now = self.get_clock().now()
                # คำนวณเวลาที่ห่างจากข้อความล่าสุด
                diff = (now - self.last_zone_msg_time).nanoseconds / 1e9
                
                if diff > self.sensor_timeout:
                    # ถ้าเงียบไปนานเกิน Timeout -> สั่งหยุดทันที
                    self.get_logger().error(f'SENSOR LOST! ({diff:.2f}s) -> EMERGENCY STOP', throttle_duration_sec=1.0)
                    self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoStopTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()