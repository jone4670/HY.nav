#!/usr/bin/env python3
"""上位机-下位机通讯测试脚本
向 /cmd_vel 发布测试速度指令，经 chassis_bridge 桥接节点打包后通过 CH340 发给 MCU
"""

import time
import rclpy
from geometry_msgs.msg import Twist

PATTERNS = [
    # 名称, vx(m/s), vy(m/s), wz(rad/s), 持续秒数
    ("前进 1m/s",           1.0,  0.0,  0.0,  2),
    ("后退 0.5m/s",        -0.5,  0.0,  0.0,  2),
    ("左移 0.5m/s",         0.0,  0.5,  0.0,  2),
    ("右移 0.5m/s",         0.0, -0.5,  0.0,  2),
    ("原地右转 1rad/s",     0.0,  0.0, -1.0,  2),
    ("原地左转 1rad/s",     0.0,  0.0,  1.0,  2),
    ("斜前方 0.5+0.3",      0.5,  0.3,  0.0,  2),
    ("前进+右转",           0.3,  0.0, -0.5,  2),
    ("停止",                0.0,  0.0,  0.0,  3),
]

def main():
    rclpy.init()
    node = rclpy.create_node('chassis_test')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    print("=" * 50)
    print("  底盘通讯测试 — 观察 MCU 收到的字节是否变化")
    print("=" * 50)

    for name, vx, vy, wz, duration in PATTERNS:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz

        pub.publish(msg)
        print(f">>> [{name}] vx={vx:+.1f} vy={vy:+.1f} wz={wz:+.1f}  ({duration}s)")

        # 50Hz 持续发布
        rate = node.create_rate(50.0)
        for _ in range(int(duration * 50)):
            pub.publish(msg)
            rate.sleep()

    print("=" * 50)
    print("  测试完成，已发布停止指令")
    print("=" * 50)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
