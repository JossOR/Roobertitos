#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_kinematics.kinematics import Robot
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import JointState

class PublicadorTrayectoria(Node):
  def __init__(self):
    super().__init__("nodo_publicador")
    # Instanciar robot
    self.robot = Robot()
    # Suscriptor para posiciones deseadas (Twist)
    self.sub_twist = self.create_subscription(Twist, 
                                              "/goals_twist",
                                              self.twist_callback,
                                              1)
    # Suscriptor para posiciones deseadas (Point)
    self.sub_ps = self.create_subscription(PointStamped, 
                                            "/clicked_point",
                                            self.ps_callback,
                                            1)
    # Publicador estado de las juntas deseado
    self.js_pub = self.create_publisher(JointState, 
                                        "/joint_states_goals",
                                        1)
    # Suscriptor a estado de las juntas actual
    self.js_sub = self.create_subscription(
      JointState,"/joint_states",
      self.js_callback, 10)

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
    self.robot.def_tray(
      th_i=(self.js_current.position[0],
            self.js_current.position[1],
            self.js_current.position[2]), 
      xi_f=(msg.linear.x, 
            msg.linear.z, 
            msg.angular.y))
    self.get_logger().info("Posición final EF: {}".format
    (self.robot.xi_m[:, self.robot.muestras - 1]))

    self.get_logger().info("Posición final juntas: {}".format
    (self.robot.th_m[:, self.robot.muestras - 1]))
    self.robot.imp_tray()
    self.robot.imp_junt()
    # Publicando trayectoria de las juntas
    self.current_pos = 0
    self.timer_pub = self.create_timer(self.robot.dt,self.timer_pub_callback)
  
  # Callback de posición deseada como PointStamped
  def ps_callback(self, msg:PointStamped):
    if self.is_moving:
      return
    self.is_moving = True
    self.get_logger().info("Posición recibida: {}".format(str(msg.point)))
    self.robot.def_tray(
      th_i=(self.js_current.position[0],
            self.js_current.position[1],
            self.js_current.position[2]), 
      xi_f=(msg.point.x, 
            msg.point.z, 
            0))
    self.get_logger().info("Posición final EF: {}".format
    (self.robot.xi_m[:, self.robot.muestras - 1]))

    self.get_logger().info("Posición final juntas: {}".format
    (self.robot.th_m[:, self.robot.muestras - 1]))
    #self.robot.imp_tray()
    #self.robot.imp_junt()
    # Publicando trayectoria de las juntas
    self.current_pos = 0
    self.timer_pub = self.create_timer(self.robot.dt,self.timer_pub_callback)


  def timer_pub_callback(self):
    # Agregar marca de tiempo
    self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
    # Agregar posición de las juntas
    self.joint_state_msg.position = [
      float(self.robot.th_m[0, self.current_pos]),
      float(self.robot.th_m[1, self.current_pos]),
      float(self.robot.th_m[2, self.current_pos])]
    # Publicar
    self.js_pub.publish(self.joint_state_msg)
    # Incrementar posición actual
    self.current_pos += 1
    # Liberar el movimiento del robot cuando llegue a la última
    if self.current_pos == (self.robot.muestras - 1):
      self.is_moving = False
      self.timer_pub.destroy()
  
  def js_callback(self, msg:JointState):
    # Guardar en una variable el estado actual de las juntas
    self.js_current = msg

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
