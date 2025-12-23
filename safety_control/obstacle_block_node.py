#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int8


class ObstacleBlockNode(Node):
    """
    อ่าน /tof_zone_array แล้วแปลงเป็น /block_state สำหรับ STM32/Recorder

    block_state (Int8):
      0 = ทางโล่ง (อนุญาตให้วิ่งต่อ / เล่น path ต่อ)
      1 = มีสิ่งกีดขวาง หรือ sensor มีปัญหา (ให้หยุดรอ)

    Logic:
      - ถ้า Center blocked -> block_state = 1 ทันที และจดเวลา last_blocked_time
      - ถ้า Center clear:
          - ถ้าเคย blocked มาก่อน -> รอ stop_delay วินาทีหลังจากครั้งสุดท้ายที่ blocked
              - ถ้ายังไม่ครบ -> block_state = 1
              - ถ้าครบแล้ว -> block_state = 0
          - ถ้าไม่เคย blocked มาก่อนเลย -> block_state = 0
      - ถ้า sensor ไม่ส่งข้อมูลเกิน sensor_timeout วินาที -> block_state = 1 (fail-safe)
    """

    def __init__(self):
        super().__init__('obstacle_block_node')

        # --- Parameters ---
        # หน่วงเวลาหลังสิ่งกีดขวางหาย ก่อนจะปลดบล็อก (วินาที)
        self.declare_parameter('stop_delay', 3.0)
        # timeout ถ้า sensor เงียบเกินนี้ให้ถือว่าอันตราย (วินาที)
        self.declare_parameter('sensor_timeout', 1.0)

        self.stop_delay = float(self.get_parameter('stop_delay').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)

        # --- State ---
        self.current_block_state = 0   # 0 = clear, 1 = blocked
        self.last_blocked_time = self.get_clock().now()
        self.last_zone_msg_time = self.get_clock().now()
        self.ever_blocked = False      # เคยเจอสิ่งกีดขวางแล้วหรือยัง

        # --- Subscribers ---
        # Array: [OuterL, InnerL, Center, InnerR, OuterR] (Int8MultiArray)
        self.create_subscription(
            Int8MultiArray,
            '/tof_zone_array',
            self.zone_callback,
            10
        )

        # --- Publishers ---
        # ส่งค่าสถานะ block ให้ udp_cmdvel_bridge → STM32
        self.block_pub = self.create_publisher(Int8, '/block_state', 10)

        # --- Timer for sensor watchdog ---
        self.create_timer(0.1, self.watchdog_callback)

        # publish ค่าเริ่มต้น (clear) 1 ครั้ง
        self._publish_block_state(0)

        self.get_logger().info('ObstacleBlockNode started.')
        self.get_logger().info(f'  stop_delay     = {self.stop_delay:.2f} s')
        self.get_logger().info(f'  sensor_timeout = {self.sensor_timeout:.2f} s')

    # ---------- Helper: publish เฉพาะตอน state เปลี่ยน ----------
    def _publish_block_state(self, new_state: int):
        new_state = 1 if new_state != 0 else 0
        if new_state == self.current_block_state:
            return
        self.current_block_state = new_state
        self.block_pub.publish(Int8(data=new_state))
        self.get_logger().info(f'[BLOCK] state -> {new_state}')

    # ---------- Callback จาก TOF ----------
    def zone_callback(self, msg: Int8MultiArray):
        # อัปเดตเวลาที่ได้รับ data ล่าสุด
        now = self.get_clock().now()
        self.last_zone_msg_time = now

        if len(msg.data) < 3:
            # format ไม่ครบ ข้าม
            return

        # Center zone = index 2
        is_center_blocked = (msg.data[2] == 1)

        if is_center_blocked:
            # เจอสิ่งกีดขวางตรงกลาง -> บล็อกทันที
            self.ever_blocked = True
            self.last_blocked_time = now
            self._publish_block_state(1)
            # log แบบ throttle กัน log รั่ว
            self.get_logger().warn(
                'Center blocked -> BLOCK=1',
                throttle_duration_sec=1.0
            )
        else:
            # Center clear
            if self.ever_blocked:
                # เคย blocked มาก่อน ต้องรอ stop_delay จาก last_blocked_time
                elapsed = (now - self.last_blocked_time).nanoseconds / 1e9
                if elapsed < self.stop_delay:
                    # ยังไม่ครบเวลา → ยัง block อยู่
                    remaining = self.stop_delay - elapsed
                    self._publish_block_state(1)
                    self.get_logger().info(
                        f'Center clear but waiting stop_delay... {remaining:.1f}s',
                        throttle_duration_sec=1.0
                    )
                else:
                    # ครบเวลาแล้ว → ปลดบล็อก
                    self._publish_block_state(0)
            else:
                # ยังไม่เคย blocked เลย → ถือว่า clear
                self._publish_block_state(0)

    # ---------- Sensor Watchdog ----------
    def watchdog_callback(self):
        now = self.get_clock().now()
        diff = (now - self.last_zone_msg_time).nanoseconds / 1e9

        if diff > self.sensor_timeout:
            # sensor เงียบเกินกำหนด -> บล็อกเพื่อความปลอดภัย
            self._publish_block_state(1)
            self.get_logger().error(
                f'SENSOR TIMEOUT ({diff:.2f}s > {self.sensor_timeout:.2f}s) -> BLOCK=1',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleBlockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
