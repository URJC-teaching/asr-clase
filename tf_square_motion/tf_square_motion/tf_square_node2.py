import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer

# Importamos las herramientas necesarias de tf_transformations
from tf_transformations import (
    quaternion_matrix, 
    translation_matrix, 
    concatenate_matrices, 
    euler_from_matrix
)

class TFSquareMover(Node):
    def __init__(self):
        super().__init__('tf_square_mover')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.control_loop)

        self.state = 'init'
        self.blref2odom = None  # Guardamos odom2blref (base_link referencia a odom)
        self.side_count = 0

    def transform_to_matrix(self, transform_stamped):
        """
        Convierte TransformStamped a una matriz homogénea 4x4 usando tf_transformations.
        """
        t = transform_stamped.transform
        
        # 1. Crear matriz de traslación (4x4)
        trans_mat = translation_matrix([t.translation.x, t.translation.y, t.translation.z])
    
        # 2. Crear matriz de rotación (4x4) desde el cuaternión
        rot_mat = quaternion_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        
        # 3. Combinarlas: Traslación * Rotación
        matrix = concatenate_matrices(trans_mat, rot_mat)
        return matrix
    
    def matrix_to_pose(self, matrix):
        """
        Extrae x, y, yaw de una matriz homogénea 4x4.
        """
        # En una matriz 4x4, la traslación está en la última columna (índice 3)
        x = matrix[0, 3]
        y = matrix[1, 3]
        
        # Usamos la librería para extraer los ángulos de Euler de la matriz
        # euler_from_matrix devuelve (roll, pitch, yaw) por defecto (ejes sxyz)
        _, _, yaw = euler_from_matrix(matrix)
        
        return x, y, yaw

    def control_loop(self):
        try:
            # Buscamos la transformación actual
            bl2odom = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            # Es normal que falle al principio mientras carga el buffer
            return

        # Inicializar la referencia si es la primera vez (prevención de seguridad)
        if self.blref2odom is None:
            self.blref2odom = bl2odom

        if self.state == 'init':
            # Guardar la transformación de referencia para comenzar el lado
            self.blref2odom = bl2odom
            self.state = 'forward'
            self.get_logger().info(f"Starting side {self.side_count + 1}")
            return

        elif self.state == 'forward':
            # Convertimos ambas transformaciones a matrices 4x4
            T_blref2odom = self.transform_to_matrix(self.blref2odom)
            T_bl2odom = self.transform_to_matrix(bl2odom)
            
            # Operación: T_relativa = inv(T_referencia) * T_actual
            # Calculamos dónde está el robot AHORA respecto a donde EMPEZÓ el movimiento
            T_bl2blref = T_bl2odom @ np.linalg.inv(T_blref2odom)

            x, y, _ = self.matrix_to_pose(T_bl2blref)
            distance = math.sqrt(x**2 + y**2)
            
            # Logueo intermitente para no saturar consola (opcional)
            # self.get_logger().info(f"Distance: {distance:.2f}")

            if distance < 1.0:  # mover 1 metro
                twist = Twist()
                twist.linear.x = 0.5
                self.publisher.publish(twist)
            else:
                self.publisher.publish(Twist())  # stop
                self.state = 'turn'
                self.blref2odom = bl2odom  # Nueva referencia para empezar a medir el giro
                self.get_logger().info(f"Finished side {self.side_count + 1}, starting turn.")
                time.sleep(0.5)

        elif self.state == 'turn':
            # Convertimos a matrices
            T_blref2odom = self.transform_to_matrix(self.blref2odom)
            T_bl2odom = self.transform_to_matrix(bl2odom)
            
            # Calculamos diferencia relativa
            T_bl2blref = T_bl2odom @ np.linalg.inv(T_blref2odom)
            
            _, _, yaw = self.matrix_to_pose(T_bl2blref)
            
            # Logueo
            # self.get_logger().info(f"Turning angle: {math.degrees(yaw):.2f} deg")

            # Giramos 90 grados (pi/2)
            if abs(yaw) < math.pi / 2:
                twist = Twist()
                twist.angular.z = 0.5 # Velocidad un poco más baja para precisión
                self.publisher.publish(twist)
            else:
                self.publisher.publish(Twist())  # stop
                self.side_count += 1
                if self.side_count >= 4:
                    self.get_logger().info("Finished square.")
                    self.state = 'done'
                else:
                    self.state = 'init' # Volvemos a init para coger referencia del siguiente lado
                time.sleep(0.5)

        elif self.state == 'done':
            # Aseguramos que el robot se detenga
            self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = TFSquareMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()