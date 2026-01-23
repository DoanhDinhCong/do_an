#!/usr/bin/env python3
"""
velocity_bridge.py - Cầu nối giao tiếp STM32 qua Serial (Đã Fix Regex)
=====================================================================
Node ROS2 giao tiếp với STM32 để điều khiển robot mecanum 4 bánh.

SỬA ĐỔI MỚI NHẤT:
-----------------
1. Regex (ENC_PAT): Đã bỏ yêu cầu phần "D=..." để khớp với log thực tế.
2. RX Loop: Tự động gán D=[0,0,0,0] vì STM32 không gửi (vận tốc vẫn được tính đúng dựa trên Delta T).

CHỨC NĂNG CHÍNH:
----------------
1. TX: Gửi "V vx vy wz" (50Hz)
2. RX: Đọc "ENC ms=... T=..." và publish JointState
"""

import time
import serial
import re
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import JointState

# =============================================================================
# REGEX PATTERN - ĐÃ SỬA
# =============================================================================
# Pattern để parse dòng encoder từ STM32
# Format thực tế nhận được: "ENC ms=1924050 T=-32098,-31924,-31564,-31083"
# Groups:
#   1: ms (timestamp)
#   2-5: T values (total ticks 4 bánh)
ENC_PAT = re.compile(
    r"ENC\s+"                           # Từ khóa "ENC" + khoảng trắng
    r"ms=(\d+)\s+"                      # ms=<số> + khoảng trắng
    r"T=([-0-9]+),([-0-9]+),([-0-9]+),([-0-9]+)"  # T=<4 số phân cách bởi dấu phẩy>
    # ĐÃ XÓA PHẦN D= VÌ STM32 KHÔNG GỬI
)


class VelocityBridgeVfmt(Node):
    """
    Node cầu nối vận tốc - Giao tiếp với STM32 điều khiển mecanum robot
    """
    
    def __init__(self):
        super().__init__('velocity_bridge')
        
        # =====================================================================
        # THAM SỐ SERIAL VÀ TRUYỀN (TX)
        # =====================================================================
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_ms', 200)
        
        self.declare_parameter('max_vx', 1.0)   # m/s
        self.declare_parameter('max_vy', 1.0)   # m/s
        self.declare_parameter('max_wz', 2.0)   # rad/s
        
        self.declare_parameter('zero_on_timeout', False)
        self.declare_parameter('echo_tx', True)
        self.declare_parameter('echo_rx', False)
        
        # =====================================================================
        # THAM SỐ ENCODER VÀ JOINT STATE
        # =====================================================================
        self.declare_parameter('ticks_per_rev', 6864.0)
        
        self.declare_parameter('wheel_joint_names', [
            'wheel_fl_joint',  # Bánh trước trái
            'wheel_fr_joint',  # Bánh trước phải
            'wheel_rr_joint',  # Bánh sau phải
            'wheel_rl_joint'   # Bánh sau trái
        ])
        
        # [FL, FR, RR, RL] - True = đảo chiều, False = giữ nguyên
        self.declare_parameter('invert_wheels', [True, True, True, True])

        # =====================================================================
        # ĐỌC GIÁ TRỊ THAM SỐ
        # =====================================================================
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('cmd_timeout_ms').value) / 1000.0
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.zero_on_timeout = bool(self.get_parameter('zero_on_timeout').value)
        self.echo_tx = bool(self.get_parameter('echo_tx').value)
        self.echo_rx = bool(self.get_parameter('echo_rx').value)

        # =====================================================================
        # LOGGING CONTROL
        # =====================================================================
        self.declare_parameter('tx_log_on_change_only', True)
        self.declare_parameter('tx_log_epsilon_v', 0.01)
        self.declare_parameter('tx_log_epsilon_w', 0.02)
        self.declare_parameter('tx_keepalive_sec', 0.0)
        
        self.tx_log_on_change_only = bool(self.get_parameter('tx_log_on_change_only').value)
        self.tx_log_eps_v = float(self.get_parameter('tx_log_epsilon_v').value)
        self.tx_log_eps_w = float(self.get_parameter('tx_log_epsilon_w').value)
        self.tx_keepalive_sec = float(self.get_parameter('tx_keepalive_sec').value)
        
        self._last_logged_cmd = None
        self._last_log_wall = time.monotonic()

        # =====================================================================
        # CHUYỂN ĐỔI ENCODER
        # =====================================================================
        tpr = float(self.get_parameter('ticks_per_rev').value)
        self.rad_per_tick = 2.0 * math.pi / tpr
        
        self.names = [str(x) for x in self.get_parameter('wheel_joint_names').value]
        self.invert = [bool(x) for x in self.get_parameter('invert_wheels').value]

        self._already_sent_zero = False

        # =====================================================================
        # KẾT NỐI SERIAL
        # =====================================================================
        self.get_logger().info(f"Đang mở cổng serial: {port} @ {baud} baud")
        
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
            time.sleep(0.2)
            self.get_logger().info("✅ Đã kết nối Serial thành công!")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Không thể mở cổng serial {port}")
            self.get_logger().error(f"   Lỗi: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi không mong đợi: {e}")
            raise

        # =====================================================================
        # PUBLISHERS
        # =====================================================================
        self.enc_line_pub = self.create_publisher(String, 'enc/line', 10)
        self.enc_total_pub = self.create_publisher(Int32MultiArray, 'enc/total', 10)
        self.enc_delta_pub = self.create_publisher(Int32MultiArray, 'enc/delta', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 20)

        # =====================================================================
        # SUBSCRIBER & TIMERS
        # =====================================================================
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd = (0.0, 0.0, 0.0)
        
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_timer(max(0.002, 1.0/self.rate), self._tick)

        # =====================================================================
        # THREAD NHẬN DỮ LIỆU (RX)
        # =====================================================================
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._rx_loop, daemon=True)
        self._thr.start()

        # =====================================================================
        # BIẾN CHO TÍNH TOÁN VELOCITY
        # =====================================================================
        self._last_ms = None
        self._last_T = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Velocity Bridge đã khởi động (Fix Regex Version)!")
        self.get_logger().info(f"   - Tần số gửi: {self.rate} Hz")
        self.get_logger().info(f"   - Xung/vòng: {tpr}")
        self.get_logger().info("=" * 60)

    # =========================================================================
    # CLEANUP
    # =========================================================================
    def destroy_node(self):
        self.get_logger().info("🛑 Đang tắt Velocity Bridge...")
        self._stop.set()
        try:
            if self._thr.is_alive(): 
                self._thr.join(timeout=0.3)
        except Exception: 
            pass
        try: 
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception: 
            pass
        return super().destroy_node()

    # =========================================================================
    # TX - TRUYỀN LỆNH
    # =========================================================================
    def _on_cmd(self, msg: Twist):
        clamp = lambda v, lo, hi: lo if v < lo else hi if v > hi else v
        
        vx = clamp(msg.linear.x,  -self.max_vx, self.max_vx)
        vy = clamp(msg.linear.y,  -self.max_vy, self.max_vy)
        wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        
        self.last_cmd = (vx, vy, wz)
        self.last_cmd_time = self.get_clock().now()
        
        if vx != 0.0 or vy != 0.0 or wz != 0.0:
            self._already_sent_zero = False

    def _send_v(self, vx, vy, wz):
        line = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"
        try:
            self.ser.write(line.encode('ascii'))
            
            # Logging logic
            if self.echo_tx:
                do_log = True
                if self.tx_log_on_change_only:
                    if self._last_logged_cmd is None:
                        do_log = True
                    else:
                        lvx, lvy, lwz = self._last_logged_cmd
                        dv = max(abs(vx - lvx), abs(vy - lvy))
                        dw = abs(wz - lwz)
                        do_log = (dv > self.tx_log_eps_v) or (dw > self.tx_log_eps_w)
                
                if not do_log and self.tx_keepalive_sec > 0.0:
                    now = time.monotonic()
                    if (now - self._last_log_wall) >= self.tx_keepalive_sec:
                        do_log = True
                
                if do_log:
                    self.get_logger().info(f"📤 TX: {line.strip()}")
                    self._last_logged_cmd = (vx, vy, wz)
                    self._last_log_wall = time.monotonic()
                    
        except Exception as e:
            self.get_logger().warn(f'❌ Lỗi gửi Serial: {e}')

    def _tick(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        
        if self.zero_on_timeout and age > self.timeout_s:
            if not self._already_sent_zero:
                self._send_v(0.0, 0.0, 0.0)
                self._already_sent_zero = True
                self.get_logger().info(f"⚠️ Cmd timeout ({age:.2f}s) - dừng robot")
        else:
            vx, vy, wz = self.last_cmd
            self._send_v(vx, vy, wz)

    # =========================================================================
    # RX - NHẬN DỮ LIỆU (ĐÃ SỬA)
    # =========================================================================
    def _rx_loop(self):
        buf = b''
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
                buf += chunk
                
                if b'\n' not in buf:
                    time.sleep(0.002)
                    continue
                
                parts = buf.split(b'\n')
                buf = parts[-1]
                
                for raw in parts[:-1]:
                    line = raw.decode(errors='ignore').strip()
                    if not line: continue
                    
                    if self.echo_rx: 
                        self.get_logger().info(f"📥 RX: {line}")
                    
                    self.enc_line_pub.publish(String(data=line))

                    # ---- PARSE REGEX ----
                    m = ENC_PAT.match(line)
                    if not m:
                        continue
                    
                    # 1. Parse Timestamp
                    ms = int(m.group(1))
                    
                    # 2. Parse Total Ticks
                    # Groups 2,3,4,5
                    T = [int(m.group(i)) for i in range(2, 6)]
                    
                    # 3. Delta Ticks (Giả lập vì STM32 không gửi)
                    d = [0, 0, 0, 0] 
                    
                    # Publish Debug Info
                    self.enc_total_pub.publish(Int32MultiArray(data=T))
                    self.enc_delta_pub.publish(Int32MultiArray(data=d))
                    
                    # ---- CALCULATE JOINT STATE ----
                    # Position
                    pos = [t * self.rad_per_tick for t in T]
                    
                    # Velocity
                    if self._last_ms is not None and hasattr(self, '_last_T'):
                        dt = (ms - self._last_ms) / 1000.0
                        if 0.0 < dt < 0.5:
                            # Tự tính Delta T để suy ra vận tốc
                            delta_T = [T[i] - self._last_T[i] for i in range(4)]
                            vel = [dT * self.rad_per_tick / dt for dT in delta_T]
                        else:
                            vel = [0.0] * 4
                    else:
                        vel = [0.0] * 4

                    self._last_T = T
                    self._last_ms = ms
                    
                    # Invert
                    pos = [(-p if inv else p) for p, inv in zip(pos, self.invert)]
                    vel = [(-v if inv else v) for v, inv in zip(vel, self.invert)]
                    
                    # Publish JointState
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = self.names
                    js.position = pos
                    js.velocity = vel
                    self.js_pub.publish(js)
                    
            except serial.SerialException as e:
                self.get_logger().error(f'❌ Lỗi RX: {e}', throttle_duration_sec=5.0)
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().debug(f'⚠️ Lỗi parse: {e}', throttle_duration_sec=5.0)
                time.sleep(0.01)

def main():
    rclpy.init()
    try:
        node = VelocityBridgeVfmt()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠️ Đã nhận Ctrl+C...")
    except Exception as e:
        print(f"❌ Lỗi nghiêm trọng: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Đã tắt")

if __name__ == '__main__':
    main()