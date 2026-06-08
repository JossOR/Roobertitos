#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class NodoHardware(Node):
  def __init__(self):
    super().__init__("nodo_hardware")
    # Publicador para mandar el estado de las juntas
    self.js_pub = self.create_publisher(
      JointState,"/joint_states",10
    )
    # Suscriptor para valores deseados
    self.j_goal_sub = self.create_subscription(
      JointState, "/joint_states_goals",
      self.goal_callback, 10
    )
    # Mensaje para publicar estado de ls juntas
    self.js_state = JointState()
    self.js_state.name = ["shoulder_joint",
                          "arm_joint",
                          "forearm_joint"]
    self.js_state.position = [0.1, 0.1, 0.1]
    # Mensaje para guardar posiciones deseadas
    self.js_goal = JointState()
    self.js_goal.name = ["shoulder_joint",
                          "arm_joint",
                          "forearm_joint"]
    self.js_goal.position = [0.1, 0.1, 0.1]
    # Timer para comunicación con el robot
    self.create_timer(0.01, self.hw_callback)

  # Callback posiciones deseadas
  def goal_callback(self, msg:JointState):
    # Lee posición deseada y enviarla al hardware
    # Por ahora sólo las guarda en una variable
    self.js_goal = msg
  # Callback timer posiciones
  # Pregunta al hardware la posición de las juntas
  def hw_callback(self):
    # Toma el valor del hardware (por ahora sólo la variable)
    self.js_state.position = self.js_goal.position
    # Actualizar mensaje
    self.js_state.header.stamp = self.get_clock().now().to_msg()
    # Publicación de estado actual
    self.js_pub.publish(self.js_state)


def main():
  try:
    rclpy.init()
    nodo_hardware = NodoHardware()
    rclpy.spin(nodo_hardware)
    rclpy.shutdown()
  except KeyboardInterrupt as e:
    print(e)
