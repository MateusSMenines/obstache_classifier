import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from rclpy.node import Node
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber

import matplotlib.pyplot as plt
import numpy as np

import math
from sklearn.cluster import DBSCAN

class Classifier(Node):

    def __init__(self):
        # Inicializando a classe base 'Node' com o nome 'Obstacle_Classifier'
        super().__init__('Obstacle_Classifier')

        # Configurando assinaturas para dados do scanner a laser e odometria
        pts_scan = Subscriber(self, LaserScan, '/scan')
        odom = Subscriber(self, Odometry, '/odom')
        
        # Sincronizando as mensagens do scanner a laser e odometria no tempo
        self.ts = ApproximateTimeSynchronizer([pts_scan, odom], 10, 0.1)
        self.ts.registerCallback(self.scan_callback)

        # Configurando publicador para o comprimento do cluster resultante
        self.publisher = self.create_publisher(Int16, "/len_cluster", 10)
        self.pub_msg = Int16()

        # Inicializando variáveis para controle de estado e visualização
        self.scan_init = False
        self.array_pts = None

        # Configurando a plotagem usando a biblioteca Matplotlib
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'ro', markersize=0.5, color="red")
        self.line_robot, = self.ax.plot([], [], 'ro', markersize=5, color="blue")

    # Função de retorno de chamada para mensagens do scanner a laser e odometria
    def scan_callback(self, msg_scan, msg_odom):
        # Inicialização do scanner a laser na primeira chamada
        if not self.scan_init:
            self.scan_init = True

        # Convertendo coordenadas polares para coordenadas do mundo
        self.polar2word(msg_scan, msg_odom)
        # Atualizando a plotagem
        self.update_plot()

    # Atualizando a plotagem com dados do scanner a laser e odometria
    def update_plot(self):
        x_data = self.array_pts[:, 0]
        y_data = self.array_pts[:, 1]
        self.line.set_data(x_data, y_data)
        self.line_robot.set_data(self.pose_odom[0], self.pose_odom[1])
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)

    # Convertendo coordenadas polares para coordenadas do mundo
    def polar2word(self, msg_scan, msg_odom):
        _, _, yaw = quaternion2rads(msg_odom.pose.pose.orientation)
        self.pose_odom = np.array([msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y])
        
        # Iterando sobre as leituras do scanner a laser e transformando as coordenadas
        for index in range(len(msg_scan.ranges)):
            if msg_scan.range_min < msg_scan.ranges[index] < msg_scan.range_max:
                angle = index * msg_scan.angle_increment
                P_S = np.array([[np.cos(angle) * msg_scan.ranges[index]],
                                [np.sin(angle) * msg_scan.ranges[index]],
                                [1]])
                M_PL = np.array([[1, 0, -0.064],
                                 [0, 1, 0],
                                 [0, 0, 1]])
                M_LW = np.array([[np.cos(yaw), -np.sin(yaw), self.pose_odom[0]],
                                 [np.sin(yaw), np.cos(yaw), self.pose_odom[1]],
                                 [0, 0, 1]])
                pose_tf = M_LW @ M_PL @ P_S
                self.array_pts = subsampling(pose_tf.T[0][0:2], self.array_pts)

        # Classificando do cluster e publicando o tamanho do maior cluster para o criterio de parada
        cluster_len = classifier(self.array_pts)
        self.pub_msg.data = cluster_len
        self.publisher.publish(self.pub_msg)


# Função para ajuste de círculo usando mínimos quadrados
def circle_fitting(cluster_points):

    """
    Ajusta um círculo aos pontos fornecidos usando mínimos quadrados.

    Parâmetros:
    - cluster_points: Matriz de pontos (x, y) pertencentes a um cluster.

    Retorna:
    Tupla (cxe, cye, re, error) representando as coordenadas do centro (cxe, cye),
    o raio (re) e o erro total do ajuste do círculo.
    """

    x = cluster_points[:,0]
    y = cluster_points[:,1]

    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum([ix ** 2 for ix in x])
    sumy2 = sum([iy ** 2 for iy in y])
    sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

    F = np.array([[sumx2, sumxy, sumx],
                  [sumxy, sumy2, sumy],
                  [sumx, sumy, len(x)]])
    G = np.array([[-sum([ix ** 3 + ix * iy ** 2 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])
    T = np.linalg.inv(F).dot(G)

    cxe = float(T[0, 0] / -2)
    cye = float(T[1, 0] / -2)
    re = math.sqrt(cxe**2 + cye**2 - T[2, 0])
    error = sum([np.hypot(cxe - ix, cye - iy) - re for (ix, iy) in zip(x, y)])

    return (cxe, cye, re, error)
    

# Função para classificação de clusters e ajuste de círculos
def classifier(array_pts):

    """
    Classifica clusters e ajusta círculos aos clusters encontrados.

    Parâmetros:
    - array_pts: Matriz de pontos tridimensionais.

    Retorna:
    Número total de pontos no maior cluster.
    """
    
    cluster_len = 0
    db = DBSCAN(eps = 0.1, min_samples = 5).fit(array_pts)
    cluster_labels = db.labels_

    for label in set(cluster_labels):
        if label == -1:
            continue
        cluster_points = array_pts[cluster_labels == label]
        if cluster_points.shape[0] > cluster_len:
            cluster_len = cluster_points.shape[0]
        try: 
            # Ajustando um círculo aos pontos do cluster
            cx, cy, re, e = circle_fitting(cluster_points)
            # Verificando a qualidade do ajuste e classificando como círculo ou retângulo
            if re < 2:
                if np.abs(e) >= 0.15:
                    print(f"position X: {cx:.2f}, Y:{cy:.2f},    class: retactangle")
                else:
                    print(f"position X: {cx:.2f}, Y:{cy:.2f},     class: circle")
        except:
            pass
    print()
    print("-----------------------------------------")
    print()
    return cluster_len


# Função para converter um quaternion em ângulos de rotação
def quaternion2rads(quaternion):

    """
    Converte um quaternion em ângulos de rotação.

    Parâmetros:
    - quaternion: Objeto quaternion contendo as componentes x, y, z e w.

    Retorna:
    Tupla (roll, pitch, yaw) representando os ângulos de rotação em torno dos eixos X, Y e Z, respectivamente.
    """

    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# Função para realizar a amostragem (subsampling) de pontos 2D
def subsampling(pose_tf, array_pts):

    """
    Realiza a amostragem de pontos 3D, evitando duplicatas.

    Parâmetros:
    - pose_tf: Posição tridimensional a ser adicionada à matriz de pontos.
    - array_pts: Matriz de pontos 3D existente.

    Retorna:
    Matriz de pontos 3D atualizada após a adição ou não adição da nova posição `pose_tf`.
    """

    pose_tf = np.array([pose_tf])
    if array_pts is None:
        array_pts = pose_tf
    else:
        pose_repeat = np.repeat(pose_tf, repeats = [array_pts.shape[0]], axis=0)
        dist = np.linalg.norm(array_pts - pose_repeat, axis = 1)
        if np.any(dist <= 0.02):
            pass
        else:
            array_pts = np.vstack((array_pts,pose_tf))

    return array_pts


# Função principal para inicializar o ROS 2 e a classe 'Classifier'
def main():

    rclpy.init()
    node = Classifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
