#!/usr/bin/env python3

import os
import pty
import socket
import select
import subprocess
import rclpy
from rclpy.node import Node

class TcpSocketServerNode(Node):
    def __init__(self):
        super().__init__('tcp_socket_server_node')
        
        # 声明 ROS 2 参数
        self.declare_parameter('lport', 8889)
        self.declare_parameter('uart_path', '/tmp/wifi2serial')
        
        # 获取 ROS 2 参数
        self.lport = self.get_parameter('lport').get_parameter_value().integer_value
        self.uart_path = self.get_parameter('uart_path').get_parameter_value().string_value

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', self.lport))
        s.listen(5)
        master, slave = pty.openpty()
        if os.path.exists(self.uart_path):
            os.remove(self.uart_path)
        os.symlink(os.ttyname(slave), self.uart_path)
        self.get_logger().info(f"TCP端口:{self.lport}，已映射到串口设备:{self.uart_path}")
        mypoll = select.poll()
        mypoll.register(master, select.POLLIN)
        try:
            while True:
                self.get_logger().info("等待接受连接..")
                client, client_address = s.accept()
                mypoll.register(client.fileno(), select.POLLIN)
                self.get_logger().info(f'来自{client_address}的连接已建立')
                is_connect = True
                try:
                    while is_connect:
                        fdlist = mypoll.poll(256)
                        for fd, event in fdlist:
                            data = os.read(fd, 256)
                            write_fd = client.fileno() if fd == master else master
                            if len(data) == 0:
                                is_connect = False
                                break
                            os.write(write_fd, data)
                except ConnectionResetError:
                    is_connect = False
                    self.get_logger().info("远程被迫断开链接")
                finally:
                    mypoll.unregister(client.fileno())
        finally:
            s.close()
            os.close(master)
            os.close(slave)
            os.remove(self.uart_path)

def main():
    rclpy.init()
    node = TcpSocketServerNode()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
