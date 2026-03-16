import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import math

class NavigationClient:
    def __init__(self, node: Node):
        self.node_ = node
        self.nav_client_ = ActionClient(self.node_, NavigateToPose, 'navigate_to_pose')
        self.node_.get_logger().debug('Cliente de navegación inicializado')

        self.goal_handle_ = None
        self.goal_active_ = False
        self.goal_done_ = False
        self.goal_success_ = False
        self.last_feedback_ = None

    def wait_for_action_server(self, timeout_sec=5.0):
        if not self.nav_client_.wait_for_server(timeout_sec=timeout_sec):
            self.node_.get_logger().error('Servidor de navegación no disponible tras espera')
            return False
        self.node_.get_logger().debug('Servidor de navegación disponible')
        return True

    def send_goal(self, target_pose: PoseStamped):
        self.goal_active_ = False
        self.goal_done_ = False
        self.goal_success_ = False
        self.goal_handle_ = None
        self.last_feedback_ = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        goal_msg.pose.header.stamp = self.node_.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        self.node_.get_logger().debug(f'Enviando objetivo: ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})')

        send_goal_future = self.nav_client_.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node_.get_logger().error('Objetivo rechazado por el servidor')
            self.goal_done_ = True
            self.goal_success_ = False
            return

        self.node_.get_logger().debug('Objetivo aceptado, navegación iniciada')
        self.goal_handle_ = goal_handle
        self.goal_active_ = True

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.last_feedback_ = feedback_msg.feedback
        self.node_.get_logger().debug(
            f'Distancia restante: {self.last_feedback_.distance_remaining:.2f} m | '
            f'Tiempo: {self.last_feedback_.navigation_time.sec} s'
        )

    def result_callback(self, future):
        result = future.result()
        self.goal_active_ = False
        self.goal_done_ = True

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node_.get_logger().debug('Navegación completada con ÉXITO')
            self.goal_success_ = True
        elif status == GoalStatus.STATUS_ABORTED:
            self.node_.get_logger().warn('Navegación ABORTADA (obstáculo o timeout)')
            self.goal_success_ = False
        elif status == GoalStatus.STATUS_CANCELED:
            self.node_.get_logger().warn('Navegación CANCELADA')
            self.goal_success_ = False
        else:
            self.node_.get_logger().error(f'Estado desconocido: {status}')
            self.goal_success_ = False

    def cancel_goal(self):
        if self.goal_handle_ and self.goal_active_:
            self.node_.get_logger().debug('Cancelando objetivo de navegación')
            self.goal_handle_.cancel_goal_async()
            self.goal_active_ = False

    def wait_for_result(self, timeout_sec=300.0):
        import time
        start_time = time.time()
        while rclpy.ok():
            if self.goal_done_:
                self.node_.get_logger().debug(f'Goal finalizado: {"ÉXITO" if self.goal_success_ else "FALLO"}')
                return self.goal_success_
            
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                self.node_.get_logger().warn(f'Timeout esperando resultado del goal ({timeout_sec} s)')
                self.cancel_goal()
                return False
                
            time.sleep(0.1)
        
        return False

    def is_goal_active(self):
        return self.goal_active_

    def is_goal_done(self):
        return self.goal_done_

    def was_goal_successful(self):
        return self.goal_success_

    def get_feedback(self):
        return self.last_feedback_

    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node_.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Convertir yaw a quaternion math simple
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = 1.0
        sp = 0.0
        cr = 1.0
        sr = 0.0

        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

        return pose
