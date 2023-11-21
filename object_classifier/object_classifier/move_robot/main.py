import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from rclpy.node import Node
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor

import numpy as np

class Controll(Node):
    """
    Classe que define um nó ROS 2 para o controle do robô com base nas leituras do scanner a laser.
    """

    def __init__(self):
        """
        Inicializa o nó de controle do robô.
        """
        super().__init__('Controll_Robot')

        # Configuração das assinaturas e publicador
        self.subscription_odom = self.create_subscription(LaserScan,
                                                          '/scan',
                                                          self.callback_scan,
                                                          10)
        self.subscription_cluster = self.create_subscription(Int16, 
                                                   '/len_cluster',
                                                   self.callback_clusters,
                                                   10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vel = Twist()
        self.dist_tolerance = 0.4
        self.ranger = 45
        self.count = 0
        self.scan_init = False

    def callback_clusters(self, msg):
        """
        Função de retorno de chamada para leituras do comprimento do cluster.
        """
        self.len_clusters = np.array(msg.data)

    def callback_scan(self, msg_scan):
        """
        Função de retorno de chamada para leituras do scanner a laser.
        """
        if not self.scan_init:
            self.scan_init = True
        self.controller(msg_scan)

    def controller(self, msg_scan):
        """
        Função de controle principal com base nas leituras do scanner a laser.
        """
        array_range = msg_scan.ranges
        range_limit = np.concatenate([array_range[-self.ranger:], 
                                      array_range[:self.ranger]])
        try:
            if self.len_clusters > self.count:
                if np.any(range_limit < self.dist_tolerance):
                    self.move_with_obstacle(range_limit)
                else:
                    self.move_robot()
                self.count += 2
            else:
                self.stop_robot()
        except:
            pass

    def move_robot(self):
        """
        Move o robô para frente.
        """
        self.vel.angular.z = 0.0
        self.vel.linear.x = 0.2
        self.publisher.publish(self.vel)

    def stop_robot(self):
        """
        Para o movimento do robô.
        """
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.publisher.publish(self.vel)

    def move_with_obstacle(self, array_range):
        """
        Move o robô com base nas leituras do scanner a laser quando obstáculos estão presentes.
        """
        if np.argmin(array_range) < len(array_range) / 2:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.2
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = -0.2
        self.publisher.publish(self.vel)

def main():
    """
    Função principal para inicializar o ROS 2 e o nó de controle do robô.
    """
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Controll()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

