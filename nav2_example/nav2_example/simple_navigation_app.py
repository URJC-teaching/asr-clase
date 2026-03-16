#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from navigation_client.navigation_client import NavigationClient

class SimpleNavigationApp(Node):
    def __init__(self):
        super().__init__('simple_navigation_app_py_node')
        self.nav_client_ = NavigationClient(self)
        self.target_pose_ = self.nav_client_.create_pose_stamped(6.0, -2.0, 0.0)
        
        self.server_ready_ = False
        self.goal_sent_ = False

        self.get_logger().info('Aplicación de navegación iniciada (Python)')
        
        self.timer_ = self.create_timer(0.5, self.control_cycle)

    def control_cycle(self):
        if not self.server_ready_:
            if self.nav_client_.wait_for_action_server(1.0):
                self.get_logger().info('Servidor disponible, preparado para navegar')
                self.server_ready_ = True
            return

        if not self.goal_sent_:
            self.get_logger().info('Enviando objetivo de navegación...')
            self.nav_client_.send_goal(self.target_pose_)
            self.goal_sent_ = True
            return

        if not self.nav_client_.is_goal_done():
            feedback = self.nav_client_.get_feedback()
            if feedback:
                t_sec = feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9
                self.get_logger().info(
                    f'\t-Distancia restante: {feedback.distance_remaining:.2f} m | '
                    f'Tiempo: {t_sec:.1f} s'
                )
            return

        if self.nav_client_.was_goal_successful():
            self.get_logger().info('Navegación completada con éxito')
        else:
            self.get_logger().warn('Navegación fallida')

        self.timer_.cancel()
        self.get_logger().info('Aplicación finalizada')

def main(args=None):
    rclpy.init(args=args)
    
    app_node = SimpleNavigationApp()
    
    try:
        rclpy.spin(app_node)
    except KeyboardInterrupt:
        pass
    finally:
        app_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
