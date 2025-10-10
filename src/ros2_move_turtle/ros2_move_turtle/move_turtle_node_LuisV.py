#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtleNode(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Estado y parámetros: inicializar ANTES de crear timers/subs/pubs
        self.current_pose = None
        self.forward_speed = 1.5
        self.stopped = False

        # Publisher hacia /turtle1/cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscriber desde /turtle1/pose
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer para publicar comandos a una frecuencia fija (ya con self.stopped definido)
        timer_period = 0.1  # segundos -> 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('move_turtle node started')

    def pose_callback(self, msg: Pose):
        # Guardamos la pose actual
        self.current_pose = msg

    def timer_callback(self):
        twist = Twist()

        if self.current_pose is None:
            # Si no tenemos pose aún, no movemos
            self.cmd_pub.publish(twist)
            return

        x = self.current_pose.x
        y = self.current_pose.y

        # Condición: si x o y es mayor que 7 -> STOP
        if x > 7.0 or y > 7.0:
            # Velocidades en cero (frena)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            # Solo mostrar el mensaje la primera vez que se detiene
            if not self.stopped:
                self.get_logger().info(f'Stopping turtle: pose x={x:.2f}, y={y:.2f}')
                self.stopped = True

        else:
            # Mover hacia adelante a velocidad constante
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtleNode()
    try:
        # Usamos un bucle controlado para evitar cierres dobles inesperados
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        # Ctrl+C aquí
        pass
    finally:
        # Usar print para evitar publicar en rosout durante el shutdown
        print('Shutting down move_turtle node')

        # Destruir nodo (proteger de excepciones)
        try:
            node.destroy_node()
        except Exception:
            pass

        # Llamar shutdown solo si rclpy aún está "ok" y envolver en try/except
        try:
            if rclpy.ok():
                rclpy.shutdown()
            # si rclpy.ok() es False, probablemente shutdown ya fue llamado: lo ignoramos
        except Exception:
            # Ignorar errores de shutdown doble
            pass


if __name__ == '__main__':
    main()
