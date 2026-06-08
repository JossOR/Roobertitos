#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_kinematics.kinematics import Robot
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class PublicadorTrayectoria(Node):
  def __init__(self):
    super().__init__("nodo_publicador")
    # Instanciar robot
    self.robot = Robot()
    # Suscriptor para posiciones deseadas
    self.sub_twist = self.create_subscription(Twist, 
                                              "/goals_twist",
                                              self.twist_callback,
                                              1)
    # Publicador estado de las juntas
    self.js_pub = self.create_publisher(JointState, 
                                        "/joint_states",
                                        1)
    # Variable de estado de movimiento
    self.is_moving = False
    # Mensaje de estado de las juntas
    self.joint_state_msg = JointState()
    self.joint_state_msg.name = ["shoulder_joint",
                                 "arm_joint",
                                 "forearm_joint"]
  # Callback de posición deseada como twist
  def twist_callback(self, msg:Twist):
    if self.is_moving:
      return
    self.is_moving = True
    self.get_logger().info("Posición recibida: {}".format(str(msg.linear)))
    self.robot.def_tray(th_i=(0.1, 0.1, 0.1), 
                        xi_f=(msg.linear.x, 
                              msg.linear.z, 
                              msg.angular.y))
    self.get_logger().info("Posición final EF: {}".format
    (self.robot.xi_m[:, self.robot.muestras - 1]))

    self.get_logger().info("Posición final juntas: {}".format
    (self.robot.th_m[:, self.robot.muestras - 1]))
    self.robot.imp_tray()
    self.robot.imp_junt()
    # Publicando última posición de las juntas
    # Agregar marca de tiempo
    self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
    # Agregar posición de las juntas
    self.joint_state_msg.position = [
      float(self.robot.th_m[0, self.robot.muestras - 1]),
      float(self.robot.th_m[1, self.robot.muestras - 1]),
      float(self.robot.th_m[2, self.robot.muestras - 1])]
    # Publicar
    self.js_pub.publish(self.joint_state_msg)
    # Liberar el movimiento del robot
    self.is_moving = False
    

def main():
  try:
    rclpy.init()
    publicador = PublicadorTrayectoria()
    rclpy.spin(publicador)
    rclpy.shutdown()
  except KeyboardInterrupt as e:
    print(e) 
if __name__ == "__main__":
  main()
