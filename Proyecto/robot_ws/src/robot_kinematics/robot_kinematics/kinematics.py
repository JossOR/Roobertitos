#!/usr/bin/env python3

import math
from typing import Optional, Sequence, Tuple

import numpy as np


class Robot:
  def __init__(self,
               l: Tuple[float, float] = (0.35, 0.30),
               base_height: float = 0.38):
    """
    Cinematica compatible con el URDF del robot RRR.

    Modelo:
      shoulder_joint : revolute en Z
      arm_joint      : revolute en Y
      forearm_joint  : revolute en Y

    Dimensiones usadas:
      H0 = 0.38 m
      L1 = 0.35 m
      L2 = 0.30 m
    """

    self.L1 = float(l[0])
    self.L2 = float(l[1])
    self.H0 = float(base_height)

    self.joint_limits = [
      (-math.pi, math.pi),       # shoulder_joint
      (-1.5708, 1.5708),         # arm_joint
      (-1.5708, 1.5708),         # forearm_joint
    ]

    self.dt = 0.0
    self.muestras = 0

    self.t_m = None

    self.xi_m = None
    self.xi_dot_m = None
    self.xi_dot_dot_m = None

    self.th_m = None
    self.th_dot_m = None
    self.th_dot_dot_m = None

  @staticmethod
  def _fit_angle_to_seed(angle: float, seed: float) -> float:
    """
    Ajusta un angulo equivalente para que quede lo mas cercano posible
    al angulo actual del robot.
    """
    while angle - seed > math.pi:
      angle -= 2.0 * math.pi

    while angle - seed < -math.pi:
      angle += 2.0 * math.pi

    return angle

  def within_limits(self, q: Sequence[float], tol: float = 1e-8) -> bool:
    """
    Verifica si las juntas estan dentro de los limites del URDF.
    """
    for qi, (lo, hi) in zip(q, self.joint_limits):
      if qi < lo - tol or qi > hi + tol:
        return False

    return True

  def fk(self, q: Sequence[float]) -> np.ndarray:
    """
    Cinematica directa.

    Entrada:
      q = [shoulder_joint, arm_joint, forearm_joint]

    Salida:
      [x, y, z] del efector final.
    """
    q1, q2, q3 = [float(v) for v in q]

    # Brazo 2R en un plano vertical local.
    r = self.L1 * math.sin(q2) + self.L2 * math.sin(q2 + q3)
    z = self.H0 + self.L1 * math.cos(q2) + self.L2 * math.cos(q2 + q3)

    # Rotacion de la base alrededor de Z.
    x = r * math.cos(q1)
    y = r * math.sin(q1)

    return np.array([x, y, z], dtype=float)

  def ik(self,
         x: float,
         y: float,
         z: float,
         seed: Optional[Sequence[float]] = None) -> np.ndarray:
    """
    Cinematica inversa analitica.

    Entrada:
      x, y, z = posicion deseada del efector final.

    Salida:
      q = [shoulder_joint, arm_joint, forearm_joint]
    """
    if seed is None:
      seed = (0.0, 0.0, 0.0)

    x = float(x)
    y = float(y)
    z = float(z)

    r = math.hypot(x, y)
    z_local = z - self.H0

    if r < 1e-9:
      q1_base = float(seed[0])
    else:
      q1_base = math.atan2(y, x)
      q1_base = self._fit_angle_to_seed(q1_base, float(seed[0]))

    D = (r*r + z_local*z_local - self.L1*self.L1 - self.L2*self.L2) / (
      2.0 * self.L1 * self.L2
    )

    # Ajuste por posibles errores numericos pequenos.
    if D > 1.0 and D < 1.0 + 1e-9:
      D = 1.0
    elif D < -1.0 and D > -1.0 - 1e-9:
      D = -1.0

    if D < -1.0 or D > 1.0:
      max_reach = self.L1 + self.L2
      min_reach = abs(self.L1 - self.L2)
      dist = math.sqrt(r*r + z_local*z_local)

      raise ValueError(
        "Punto fuera del alcance geometrico. "
        f"dist={dist:.3f} m, "
        f"rango=[{min_reach:.3f}, {max_reach:.3f}] m, "
        f"objetivo=({x:.3f}, {y:.3f}, {z:.3f})"
      )

    candidates = []
    root = math.sqrt(max(0.0, 1.0 - D*D))

    for sign in (1.0, -1.0):
      q3 = math.atan2(sign * root, D)

      q2 = math.atan2(r, z_local) - math.atan2(
        self.L2 * math.sin(q3),
        self.L1 + self.L2 * math.cos(q3)
      )

      q = np.array([q1_base, q2, q3], dtype=float)

      if self.within_limits(q):
        cost = float(np.sum((q - np.array(seed, dtype=float))**2))
        candidates.append((cost, q))

    if not candidates:
      raise ValueError(
        "El punto es alcanzable geometricamente, pero no con los limites "
        "articulares del URDF. "
        f"objetivo=({x:.3f}, {y:.3f}, {z:.3f})"
      )

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]

  def closest_reachable_point(self,
                              x: float,
                              y: float,
                              z: float,
                              seed: Optional[Sequence[float]] = None,
                              n_q2: int = 121,
                              n_q3: int = 121):
    """
    Busca el punto alcanzable mas cercano al objetivo.

    Esto sirve para que al publicar un punto fuera del workspace,
    el robot no falle, sino que se mueva al punto valido mas cercano.
    """
    if seed is None:
      seed = (0.0, 0.0, 0.0)

    target = np.array([float(x), float(y), float(z)], dtype=float)

    r = math.hypot(x, y)

    if r < 1e-9:
      q1 = float(seed[0])
    else:
      q1 = math.atan2(y, x)
      q1 = self._fit_angle_to_seed(q1, float(seed[0]))

    q1 = min(max(q1, self.joint_limits[0][0]), self.joint_limits[0][1])

    q2_min, q2_max = self.joint_limits[1]
    q3_min, q3_max = self.joint_limits[2]

    best_point = None
    best_q = None
    best_cost = float("inf")

    q2_values = np.linspace(q2_min, q2_max, n_q2)
    q3_values = np.linspace(q3_min, q3_max, n_q3)

    for q2 in q2_values:
      for q3 in q3_values:
        q = np.array([q1, q2, q3], dtype=float)
        p = self.fk(q)

        dist = float(np.linalg.norm(p - target))
        posture_cost = 0.02 * float(np.linalg.norm(q - np.array(seed, dtype=float)))
        cost = dist + posture_cost

        if cost < best_cost:
          best_cost = cost
          best_point = p
          best_q = q

    real_distance = float(np.linalg.norm(best_point - target))

    return best_point, best_q, real_distance

  def def_tray(self,
               t_f: float = 2.0,
               frec: float = 15.0,
               th_i: Sequence[float] = (0.1, 0.1, 0.1),
               xi_f: Sequence[float] = (0.30, 0.0, 0.95)):
    """
    Genera trayectoria suave con polinomio de quinto grado.

    Entrada:
      th_i = posicion inicial de juntas
      xi_f = posicion final deseada del efector [x, y, z]
    """
    th_i = np.array(th_i, dtype=float)

    if len(xi_f) != 3:
      raise ValueError("xi_f debe tener 3 valores: (x, y, z).")

    xi_f = np.array(xi_f, dtype=float)

    th_f = self.ik(
      xi_f[0],
      xi_f[1],
      xi_f[2],
      seed=th_i
    )

    self.dt = 1.0 / float(frec)
    self.muestras = int(round(float(t_f) * float(frec))) + 1

    self.t_m = np.zeros((1, self.muestras), dtype=float)

    self.xi_m = np.zeros((3, self.muestras), dtype=float)
    self.xi_dot_m = np.zeros((3, self.muestras), dtype=float)
    self.xi_dot_dot_m = np.zeros((3, self.muestras), dtype=float)

    self.th_m = np.zeros((3, self.muestras), dtype=float)
    self.th_dot_m = np.zeros((3, self.muestras), dtype=float)
    self.th_dot_dot_m = np.zeros((3, self.muestras), dtype=float)

    delta_th = th_f - th_i

    for i in range(self.muestras):
      t = self.dt * i
      self.t_m[0, i] = t

      s = min(max(t / float(t_f), 0.0), 1.0)

      # Polinomio quintico:
      # lambda(0)=0, lambda(tf)=1,
      # velocidad y aceleracion inicial/final en cero.
      lam = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
      lam_dot = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / float(t_f)
      lam_dot_dot = (60.0*s - 180.0*s**2 + 120.0*s**3) / (float(t_f)**2)

      q = th_i + delta_th * lam

      self.th_m[:, i] = q
      self.th_dot_m[:, i] = delta_th * lam_dot
      self.th_dot_dot_m[:, i] = delta_th * lam_dot_dot

      self.xi_m[:, i] = self.fk(q)

    # Derivadas cartesianas numericas.
    if self.muestras > 1:
      self.xi_dot_m[:, 1:] = np.diff(self.xi_m, axis=1) / self.dt
      self.xi_dot_m[:, 0] = self.xi_dot_m[:, 1]

      self.xi_dot_dot_m[:, 1:] = np.diff(self.xi_dot_m, axis=1) / self.dt
      self.xi_dot_dot_m[:, 0] = self.xi_dot_dot_m[:, 1]

    print("Objetivo cartesiano [x, y, z]:", xi_f)
    print("FK inicial [x, y, z]:", self.xi_m[:, 0])
    print("FK final   [x, y, z]:", self.xi_m[:, -1])
    print("Juntas finales [shoulder, arm, forearm]:", self.th_m[:, -1])

  def imp_tray(self, show: bool = True):
    """
    Grafica posicion, velocidad y aceleracion del efector final.
    Basada en el formato del codigo preliminar 3D.
    """
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(nrows=3, ncols=3, figsize=(12, 8))
    fig.suptitle("Dinamica del Efector Final (Espacio Cartesiano 3D)")

    componentes = ["Eje X", "Eje Y", "Eje Z"]
    colores = ["red", "green", "blue"]

    for i in range(3):
      axs[0, i].set_title(f"Posicion {componentes[i]}")
      axs[0, i].plot(self.t_m.T, self.xi_m[i, :].T, color=colores[i])
      axs[0, i].grid(True)

      axs[1, i].set_title(f"Velocidad {componentes[i]}")
      axs[1, i].plot(
        self.t_m.T,
        self.xi_dot_m[i, :].T,
        color=colores[i],
        linestyle="--"
      )
      axs[1, i].grid(True)

      axs[2, i].set_title(f"Aceleracion {componentes[i]}")
      axs[2, i].plot(
        self.t_m.T,
        self.xi_dot_dot_m[i, :].T,
        color=colores[i],
        linestyle=":"
      )
      axs[2, i].grid(True)

    for ax in axs[2, :]:
      ax.set_xlabel("Tiempo [s]")

    axs[0, 0].set_ylabel("Posicion [m]")
    axs[1, 0].set_ylabel("Velocidad [m/s]")
    axs[2, 0].set_ylabel("Aceleracion [m/s^2]")

    plt.tight_layout()

    if show:
      plt.show()

    return fig

  def imp_junt(self, show: bool = True):
    """
    Grafica posicion, velocidad y aceleracion de las juntas.
    Basada en el formato del codigo preliminar 3D.
    """
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(nrows=3, ncols=3, figsize=(12, 8))
    fig.suptitle("Dinamica de las Juntas (Espacio Articular)")

    juntas = [
      "Hombro / Base (shoulder)",
      "Brazo (arm)",
      "Antebrazo (forearm)"
    ]

    colores = ["orange", "purple", "brown"]

    for i in range(3):
      axs[0, i].set_title(f"Posicion {juntas[i]}")
      axs[0, i].plot(self.t_m.T, self.th_m[i, :].T, color=colores[i])
      axs[0, i].grid(True)

      axs[1, i].set_title(f"Velocidad {juntas[i]}")
      axs[1, i].plot(
        self.t_m.T,
        self.th_dot_m[i, :].T,
        color=colores[i],
        linestyle="--"
      )
      axs[1, i].grid(True)

      axs[2, i].set_title(f"Aceleracion {juntas[i]}")
      axs[2, i].plot(
        self.t_m.T,
        self.th_dot_dot_m[i, :].T,
        color=colores[i],
        linestyle=":"
      )
      axs[2, i].grid(True)

    for ax in axs[2, :]:
      ax.set_xlabel("Tiempo [s]")

    axs[0, 0].set_ylabel("Posicion [rad]")
    axs[1, 0].set_ylabel("Velocidad [rad/s]")
    axs[2, 0].set_ylabel("Aceleracion [rad/s^2]")

    plt.tight_layout()

    if show:
      plt.show()

    return fig

  def mostrar_graficas(self):
    """
    Crea las dos figuras y abre Matplotlib una sola vez.

    Esto evita que ROS deje las ventanas congeladas o que solo aparezca
    una de las dos figuras.
    """
    import matplotlib.pyplot as plt

    self.imp_tray(show=False)
    self.imp_junt(show=False)

    plt.show()


def main():
  robot = Robot()

  q = (0.1, 0.1, 0.1)

  print("Validacion FK con q=(0.1, 0.1, 0.1):")
  print(robot.fk(q))

  robot.def_tray(
    th_i=q,
    xi_f=(0.30, 0.0, 0.95)
  )

  robot.mostrar_graficas()


if __name__ == "__main__":
  main()