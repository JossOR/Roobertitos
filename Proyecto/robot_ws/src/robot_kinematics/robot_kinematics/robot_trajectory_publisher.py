#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_kinematics.kinematics import Robot

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class PublicadorTrayectoria(Node):
  def __init__(self):
    super().__init__("nodo_publicador_trayectoria")

    self.robot = Robot()

    self.sub_twist = self.create_subscription(
      Twist,
      "/goals_twist",
      self.twist_callback,
      1
    )

    self.js_pub = self.create_publisher(
      JointState,
      "/joint_states",
      1
    )

    self.js_sub = self.create_subscription(
      JointState,
      "/joint_states",
      self.js_callback,
      10
    )

    self.is_moving = False
    self.timer_pub = None
    self.current_pos = 0

    self.joint_names = [
      "shoulder_joint",
      "arm_joint",
      "forearm_joint"
    ]

    self.joint_state_msg = JointState()
    self.joint_state_msg.name = self.joint_names

    self.current_joints = [0.1, 0.1, 0.1]

    self.get_logger().info(
      "Trajectory publisher listo. Envia objetivos [x,y,z] por /goals_twist."
    )

    self.get_logger().info(
      "Ejemplo: ros2 topic pub --once /goals_twist geometry_msgs/msg/Twist "
      "\"{linear: {x: 0.30, y: 0.0, z: 0.95}}\""
    )

  def js_callback(self, msg: JointState):
    """
    Actualiza la posicion actual de las juntas.

    Se usa el nombre de cada junta para evitar problemas si el orden
    del JointState cambia o si aparecen mas juntas.
    """
    if self.is_moving:
      return

    if len(msg.name) == 0 or len(msg.position) == 0:
      return

    new_joints = list(self.current_joints)

    for i, joint_name in enumerate(self.joint_names):
      if joint_name in msg.name:
        idx = msg.name.index(joint_name)

        if idx < len(msg.position):
          new_joints[i] = float(msg.position[idx])

    self.current_joints = new_joints

  def twist_callback(self, msg: Twist):
    if self.is_moving:
      self.get_logger().warn(
        "Ignorando objetivo: ya hay una trayectoria activa."
      )
      return

    x = float(msg.linear.x)
    y = float(msg.linear.y)
    z = float(msg.linear.z)

    self.get_logger().info(
      "Objetivo recibido por /goals_twist: x={:.3f}, y={:.3f}, z={:.3f}".format(
        x,
        y,
        z
      )
    )

    self.start_trajectory(x, y, z)

  def start_trajectory(self, x, y, z):
    target = (
      float(x),
      float(y),
      float(z)
    )

    was_projected = False

    try:
      self.robot.ik(
        target[0],
        target[1],
        target[2],
        seed=tuple(self.current_joints)
      )

    except ValueError as exc:
      self.get_logger().warn(str(exc))
      self.get_logger().warn(
        "El punto esta fuera del workspace. "
        "Buscando punto alcanzable mas cercano..."
      )

      projected_point, projected_q, dist = self.robot.closest_reachable_point(
        target[0],
        target[1],
        target[2],
        seed=tuple(self.current_joints)
      )

      target = (
        float(projected_point[0]),
        float(projected_point[1]),
        float(projected_point[2])
      )

      was_projected = True

      self.get_logger().warn(
        "Punto proyectado: x={:.3f}, y={:.3f}, z={:.3f}. "
        "Distancia al objetivo original: {:.3f} m".format(
          target[0],
          target[1],
          target[2],
          dist
        )
      )

    try:
      self.robot.def_tray(
        t_f=2.0,
        frec=30.0,
        th_i=tuple(self.current_joints),
        xi_f=target
      )

    except ValueError as exc:
      self.get_logger().warn(str(exc))
      self.get_logger().warn(
        "No se pudo generar trayectoria."
      )
      return

    self.get_logger().info(
      "FK final calculada: x={:.3f}, y={:.3f}, z={:.3f}".format(
        float(self.robot.xi_m[0, -1]),
        float(self.robot.xi_m[1, -1]),
        float(self.robot.xi_m[2, -1])
      )
    )

    self.get_logger().info(
      "Juntas finales: shoulder={:.3f}, arm={:.3f}, forearm={:.3f}".format(
        float(self.robot.th_m[0, -1]),
        float(self.robot.th_m[1, -1]),
        float(self.robot.th_m[2, -1])
      )
    )

    if was_projected:
      self.get_logger().warn(
        "Nota: el robot no ira al punto exacto publicado, "
        "sino al punto alcanzable mas cercano."
      )

    self.is_moving = True
    self.current_pos = 0

    if self.timer_pub is not None:
      self.timer_pub.destroy()

    self.timer_pub = self.create_timer(
      self.robot.dt,
      self.timer_pub_callback
    )

  def publish_joint_state(self):
    self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
    self.joint_state_msg.position = list(self.current_joints)
    self.js_pub.publish(self.joint_state_msg)

  def timer_pub_callback(self):
    if self.current_pos >= self.robot.muestras:
      self.finish_trajectory()
      return

    self.current_joints = [
      float(self.robot.th_m[0, self.current_pos]),
      float(self.robot.th_m[1, self.current_pos]),
      float(self.robot.th_m[2, self.current_pos])
    ]

    self.publish_joint_state()

    self.current_pos += 1

    if self.current_pos >= self.robot.muestras:
      self.finish_trajectory()

  def finish_trajectory(self):
    self.is_moving = False

    if self.timer_pub is not None:
      self.timer_pub.destroy()
      self.timer_pub = None

    self.get_logger().info(
      "Trayectoria terminada. Mostrando graficas..."
    )

    self.robot.mostrar_graficas()

    self.get_logger().info(
      "Graficas cerradas. Puedes publicar otro objetivo."
    )


def main():
  rclpy.init()

  publicador = PublicadorTrayectoria()

  try:
    rclpy.spin(publicador)
  except KeyboardInterrupt:
    pass
  finally:
    publicador.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
