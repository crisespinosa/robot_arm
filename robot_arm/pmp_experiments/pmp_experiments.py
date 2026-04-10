#!/usr/bin/env python3
"""
Experimentos con Planificación de Trayectorias basada en el Principio del Máximo de Pontryagin (PMP)
para el manipulador UR5e.

Experimentos:
  1. Perfiles de trayectoria PMP minimum-jerk (q, dq, ddq, jerk, costados, costo)
  2. Efecto de la duración T sobre el costo de jerk
  3. Condiciones de frontera no nulas (velocidades iniciales/finales)
  4. Trayectorias multi-punto (waypoints)
  5. Comparación PMP vs perfil trapezoidal de velocidad

Autor: Generado automáticamente para la tarea del curso
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import os

OUTPUT_DIR = "/sessions/lucid-clever-fermi/mnt/robot_arm/robot_arm/pmp_experiments"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============================================================
# Implementación PMP minimum-jerk (Python, equivalente a trajectory.hpp)
# ============================================================

def quintic_coeffs(q0, v0, a0, q1, v1, a1, T):
    """Coeficientes del polinomio de 5to grado con condiciones de frontera generales."""
    A = np.zeros((6, 6))
    b = np.zeros(6)
    # q(0) = q0
    A[0, 0] = 1.0; b[0] = q0
    # dq(0) = v0
    A[1, 1] = 1.0; b[1] = v0
    # ddq(0) = a0
    A[2, 2] = 2.0; b[2] = a0
    # q(T) = q1
    A[3] = [1, T, T**2, T**3, T**4, T**5]; b[3] = q1
    # dq(T) = v1
    A[4] = [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4]; b[4] = v1
    # ddq(T) = a1
    A[5] = [0, 0, 2, 6*T, 12*T**2, 20*T**3]; b[5] = a1
    return np.linalg.solve(A, b)


def plan_pmp_minimum_jerk(q0, dq0, ddq0, q1, dq1, ddq1, T, dt=0.001):
    """
    Planifica trayectoria minimum-jerk usando PMP.
    Retorna diccionario con t, q, dq, ddq, u (jerk), lambda1-3, J_acc.
    """
    dof = len(q0)
    N = max(2, int(np.ceil(T / dt)))
    dt_eff = T / N

    coeffs = []
    for i in range(dof):
        coeffs.append(quintic_coeffs(q0[i], dq0[i], ddq0[i], q1[i], dq1[i], ddq1[i], T))

    t_arr = np.linspace(0, T, N + 1)
    q_arr = np.zeros((N + 1, dof))
    dq_arr = np.zeros((N + 1, dof))
    ddq_arr = np.zeros((N + 1, dof))
    u_arr = np.zeros((N + 1, dof))
    lam1 = np.zeros((N + 1, dof))
    lam2 = np.zeros((N + 1, dof))
    lam3 = np.zeros((N + 1, dof))
    J_acc = np.zeros(N + 1)

    cost = 0.0
    for k in range(N + 1):
        t = t_arr[k]
        tp = np.array([1, t, t**2, t**3, t**4, t**5])
        tp_d1 = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
        tp_d2 = np.array([0, 0, 2, 6*t, 12*t**2, 20*t**3])
        tp_d3 = np.array([0, 0, 0, 6, 24*t, 60*t**2])

        for i in range(dof):
            a = coeffs[i]
            q_arr[k, i] = a @ tp
            dq_arr[k, i] = a @ tp_d1
            ddq_arr[k, i] = a @ tp_d2
            u_arr[k, i] = a @ tp_d3

            # Variables coestado (adjuntas) PMP
            lam3[k, i] = -u_arr[k, i]  # u* = -lambda3
            du_dt = 24*a[4] + 120*a[5]*t
            d2u_dt2 = 120*a[5]
            lam2[k, i] = du_dt
            lam1[k, i] = -d2u_dt2

        u2 = np.sum(u_arr[k]**2)
        if k > 0:
            cost += 0.5 * u2 * dt_eff
        J_acc[k] = cost

    return {
        't': t_arr, 'q': q_arr, 'dq': dq_arr, 'ddq': ddq_arr,
        'u': u_arr, 'lambda1': lam1, 'lambda2': lam2, 'lambda3': lam3,
        'J_acc': J_acc, 'J_total': cost
    }


def trapezoidal_velocity_profile(q0, q1, T, dt=0.001, accel_fraction=0.3):
    """Perfil trapezoidal de velocidad para comparación."""
    dof = len(q0)
    N = max(2, int(np.ceil(T / dt)))
    t_arr = np.linspace(0, T, N + 1)

    q_arr = np.zeros((N + 1, dof))
    dq_arr = np.zeros((N + 1, dof))
    ddq_arr = np.zeros((N + 1, dof))
    u_arr = np.zeros((N + 1, dof))  # jerk

    ta = accel_fraction * T  # acceleration time
    tc = T - 2 * ta  # constant velocity time

    for i in range(dof):
        delta = q1[i] - q0[i]
        v_max = delta / (T - ta)
        a_max = v_max / ta

        for k, t in enumerate(t_arr):
            if t <= ta:
                # Acceleration phase
                q_arr[k, i] = q0[i] + 0.5 * a_max * t**2
                dq_arr[k, i] = a_max * t
                ddq_arr[k, i] = a_max
            elif t <= ta + tc:
                # Constant velocity
                q_arr[k, i] = q0[i] + 0.5 * a_max * ta**2 + v_max * (t - ta)
                dq_arr[k, i] = v_max
                ddq_arr[k, i] = 0.0
            else:
                # Deceleration
                t_dec = t - ta - tc
                q_arr[k, i] = q0[i] + 0.5 * a_max * ta**2 + v_max * tc + v_max * t_dec - 0.5 * a_max * t_dec**2
                dq_arr[k, i] = v_max - a_max * t_dec
                ddq_arr[k, i] = -a_max

    # Compute jerk (numerical differentiation of ddq)
    for k in range(1, N + 1):
        u_arr[k] = (ddq_arr[k] - ddq_arr[k-1]) / (t_arr[k] - t_arr[k-1])

    J_acc = np.zeros(N + 1)
    cost = 0.0
    dt_eff = T / N
    for k in range(1, N + 1):
        cost += 0.5 * np.sum(u_arr[k]**2) * dt_eff
        J_acc[k] = cost

    return {
        't': t_arr, 'q': q_arr, 'dq': dq_arr, 'ddq': ddq_arr,
        'u': u_arr, 'J_acc': J_acc, 'J_total': cost
    }


# ============================================================
# Configuraciones del UR5e
# ============================================================

UR5E_JOINT_NAMES = ['Base', 'Hombro', 'Codo', 'Muñeca 1', 'Muñeca 2', 'Muñeca 3']
UR5E_TAU_MAX = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

# Configuraciones típicas (radianes)
Q_HOME = np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
Q_TARGET1 = np.array([np.pi/4, -np.pi/3, np.pi/6, -np.pi/4, np.pi/3, np.pi/6])
Q_TARGET2 = np.array([-np.pi/6, -np.pi/4, np.pi/4, -np.pi/3, -np.pi/6, np.pi/4])

plt.rcParams.update({
    'figure.dpi': 150,
    'font.size': 10,
    'axes.titlesize': 12,
    'axes.labelsize': 11,
    'legend.fontsize': 9,
    'lines.linewidth': 1.5,
    'figure.facecolor': 'white',
})

COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

# ============================================================
# Experimento 1: Perfiles de Trayectoria PMP
# ============================================================
print("=== Experimento 1: Perfiles de trayectoria PMP ===")

q0 = Q_HOME
q1 = Q_TARGET1
T = 3.0

zeros6 = np.zeros(6)
traj = plan_pmp_minimum_jerk(q0, zeros6, zeros6, q1, zeros6, zeros6, T, dt=0.001)

fig, axes = plt.subplots(3, 2, figsize=(14, 12))
fig.suptitle('Experimento 1: Perfiles de Trayectoria PMP Minimum-Jerk\n(UR5e, T=3s, reposo→reposo)', fontsize=14, fontweight='bold')

# q(t)
ax = axes[0, 0]
for i in range(6):
    ax.plot(traj['t'], np.degrees(traj['q'][:, i]), color=COLORS[i], label=UR5E_JOINT_NAMES[i])
ax.set_ylabel('Posición (°)')
ax.set_title('a) Posición articular q(t)')
ax.legend(loc='best', ncol=2)
ax.grid(True, alpha=0.3)

# dq(t)
ax = axes[0, 1]
for i in range(6):
    ax.plot(traj['t'], np.degrees(traj['dq'][:, i]), color=COLORS[i], label=UR5E_JOINT_NAMES[i])
ax.set_ylabel('Velocidad (°/s)')
ax.set_title('b) Velocidad articular dq(t)')
ax.legend(loc='best', ncol=2)
ax.grid(True, alpha=0.3)

# ddq(t)
ax = axes[1, 0]
for i in range(6):
    ax.plot(traj['t'], np.degrees(traj['ddq'][:, i]), color=COLORS[i], label=UR5E_JOINT_NAMES[i])
ax.set_ylabel('Aceleración (°/s²)')
ax.set_title('c) Aceleración articular ddq(t)')
ax.legend(loc='best', ncol=2)
ax.grid(True, alpha=0.3)

# u(t) = jerk
ax = axes[1, 1]
for i in range(6):
    ax.plot(traj['t'], traj['u'][:, i], color=COLORS[i], label=UR5E_JOINT_NAMES[i])
ax.set_ylabel('Jerk (rad/s³)')
ax.set_title('d) Control óptimo u*(t) = jerk')
ax.legend(loc='best', ncol=2)
ax.grid(True, alpha=0.3)

# Variables coestado lambda3
ax = axes[2, 0]
for i in range(6):
    ax.plot(traj['t'], traj['lambda3'][:, i], color=COLORS[i], label=f'λ₃ {UR5E_JOINT_NAMES[i]}')
ax.set_ylabel('λ₃(t)')
ax.set_xlabel('Tiempo (s)')
ax.set_title('e) Variable coestado λ₃(t) = -u*(t)')
ax.legend(loc='best', ncol=2, fontsize=8)
ax.grid(True, alpha=0.3)

# Costo acumulado
ax = axes[2, 1]
ax.plot(traj['t'], traj['J_acc'], 'k-', linewidth=2)
ax.set_ylabel('J(t)')
ax.set_xlabel('Tiempo (s)')
ax.set_title(f'f) Costo acumulado J = ∫½||u||²dt = {traj["J_total"]:.4f}')
ax.grid(True, alpha=0.3)
ax.fill_between(traj['t'], 0, traj['J_acc'], alpha=0.15, color='steelblue')

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, '01_pmp_profiles.png'), dpi=150, bbox_inches='tight')
plt.close()
print(f"  Costo total J = {traj['J_total']:.6f}")

# ============================================================
# Experimento 2: Efecto de la duración T sobre el costo
# ============================================================
print("\n=== Experimento 2: Efecto de T sobre el costo de jerk ===")

T_values = np.linspace(0.5, 8.0, 30)
J_values = []
max_jerk_values = []
max_vel_values = []
max_accel_values = []

for T_val in T_values:
    res = plan_pmp_minimum_jerk(q0, zeros6, zeros6, q1, zeros6, zeros6, T_val, dt=0.002)
    J_values.append(res['J_total'])
    max_jerk_values.append(np.max(np.abs(res['u'])))
    max_vel_values.append(np.max(np.abs(res['dq'])))
    max_accel_values.append(np.max(np.abs(res['ddq'])))

J_values = np.array(J_values)
max_jerk_values = np.array(max_jerk_values)
max_vel_values = np.array(max_vel_values)
max_accel_values = np.array(max_accel_values)

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Experimento 2: Efecto de la Duración T sobre la Trayectoria PMP', fontsize=14, fontweight='bold')

ax = axes[0, 0]
ax.plot(T_values, J_values, 'b-o', markersize=4)
ax.set_ylabel('Costo J')
ax.set_xlabel('Duración T (s)')
ax.set_title('a) Costo funcional J vs T')
ax.set_yscale('log')
ax.grid(True, alpha=0.3)

ax = axes[0, 1]
ax.plot(T_values, max_jerk_values, 'r-o', markersize=4)
ax.set_ylabel('|u|_max (rad/s³)')
ax.set_xlabel('Duración T (s)')
ax.set_title('b) Jerk máximo vs T')
ax.grid(True, alpha=0.3)

ax = axes[1, 0]
ax.plot(T_values, np.degrees(max_vel_values), 'g-o', markersize=4)
ax.set_ylabel('|dq|_max (°/s)')
ax.set_xlabel('Duración T (s)')
ax.set_title('c) Velocidad máxima vs T')
# Add UR5e velocity limits
ax.axhline(y=np.degrees(np.pi), color='r', linestyle='--', alpha=0.5, label='Límite juntas 1-3')
ax.axhline(y=np.degrees(2*np.pi), color='orange', linestyle='--', alpha=0.5, label='Límite juntas 4-6')
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[1, 1]
ax.plot(T_values, np.degrees(max_accel_values), 'm-o', markersize=4)
ax.set_ylabel('|ddq|_max (°/s²)')
ax.set_xlabel('Duración T (s)')
ax.set_title('d) Aceleración máxima vs T')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, '02_effect_of_T.png'), dpi=150, bbox_inches='tight')
plt.close()

# Print table
print(f"  {'T (s)':>6}  {'J':>12}  {'|u|_max':>10}  {'|dq|_max (°/s)':>15}  {'|ddq|_max (°/s²)':>18}")
for i in range(0, len(T_values), 5):
    print(f"  {T_values[i]:6.1f}  {J_values[i]:12.4f}  {max_jerk_values[i]:10.4f}  {np.degrees(max_vel_values[i]):15.2f}  {np.degrees(max_accel_values[i]):18.2f}")

# ============================================================
# Experimento 3: Condiciones de frontera no nulas
# ============================================================
print("\n=== Experimento 3: Condiciones de frontera no nulas ===")

T = 3.0
conditions = [
    ("Reposo → Reposo", zeros6, zeros6, zeros6, zeros6),
    ("v₀=0.5 → Reposo", np.full(6, 0.5), zeros6, zeros6, zeros6),
    ("Reposo → v₁=0.5", zeros6, zeros6, np.full(6, 0.5), zeros6),
    ("v₀=0.5 → v₁=-0.5", np.full(6, 0.5), zeros6, np.full(6, -0.5), zeros6),
]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Experimento 3: Efecto de Condiciones de Frontera en PMP\n(Junta 1 — Base, T=3s)', fontsize=14, fontweight='bold')

joint_idx = 0  # Show joint 1 (Base) for clarity

for idx, (name, dq0_bc, ddq0_bc, dq1_bc, ddq1_bc) in enumerate(conditions):
    res = plan_pmp_minimum_jerk(q0, dq0_bc, ddq0_bc, q1, dq1_bc, ddq1_bc, T)

    ax_q = axes[0, 0]
    ax_dq = axes[0, 1]
    ax_ddq = axes[1, 0]
    ax_u = axes[1, 1]

    ax_q.plot(res['t'], np.degrees(res['q'][:, joint_idx]), color=COLORS[idx], label=name)
    ax_dq.plot(res['t'], np.degrees(res['dq'][:, joint_idx]), color=COLORS[idx], label=name)
    ax_ddq.plot(res['t'], np.degrees(res['ddq'][:, joint_idx]), color=COLORS[idx], label=name)
    ax_u.plot(res['t'], res['u'][:, joint_idx], color=COLORS[idx], label=name)

    print(f"  {name:25s}  J = {res['J_total']:.6f}")

axes[0, 0].set_title('a) Posición q₁(t)'); axes[0, 0].set_ylabel('Posición (°)')
axes[0, 1].set_title('b) Velocidad dq₁(t)'); axes[0, 1].set_ylabel('Velocidad (°/s)')
axes[1, 0].set_title('c) Aceleración ddq₁(t)'); axes[1, 0].set_ylabel('Aceleración (°/s²)')
axes[1, 1].set_title('d) Control u₁*(t) = jerk'); axes[1, 1].set_ylabel('Jerk (rad/s³)')

for ax in axes.flat:
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Tiempo (s)')

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, '03_boundary_conditions.png'), dpi=150, bbox_inches='tight')
plt.close()

# ============================================================
# Experimento 4: Trayectorias multi-punto (waypoints)
# ============================================================
print("\n=== Experimento 4: Trayectorias multi-punto ===")

waypoints = [Q_HOME, Q_TARGET1, Q_TARGET2, Q_HOME]
T_segments = [2.0, 2.0, 2.0]

# Build full trajectory through waypoints
t_full = []
q_full = []
dq_full = []
ddq_full = []
u_full = []
J_segments = []
t_offset = 0.0

for seg in range(len(T_segments)):
    T_seg = T_segments[seg]
    q_start = waypoints[seg]
    q_end = waypoints[seg + 1]

    res = plan_pmp_minimum_jerk(q_start, zeros6, zeros6, q_end, zeros6, zeros6, T_seg, dt=0.002)

    # Skip first point for segments after the first to avoid duplicates
    start_idx = 1 if seg > 0 else 0
    t_full.extend(res['t'][start_idx:] + t_offset)
    q_full.extend(res['q'][start_idx:])
    dq_full.extend(res['dq'][start_idx:])
    ddq_full.extend(res['ddq'][start_idx:])
    u_full.extend(res['u'][start_idx:])
    J_segments.append(res['J_total'])
    t_offset += T_seg

t_full = np.array(t_full)
q_full = np.array(q_full)
dq_full = np.array(dq_full)
u_full = np.array(u_full)

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Experimento 4: Trayectoria Multi-Punto PMP\n(3 segmentos: Home→Target1→Target2→Home)', fontsize=14, fontweight='bold')

# Mark waypoint transitions
wp_times = [0] + list(np.cumsum(T_segments))

ax = axes[0, 0]
for i in range(6):
    ax.plot(t_full, np.degrees(q_full[:, i]), color=COLORS[i], label=UR5E_JOINT_NAMES[i])
for wt in wp_times[1:-1]:
    ax.axvline(x=wt, color='gray', linestyle='--', alpha=0.5)
ax.set_title('a) Posición articular q(t)')
ax.set_ylabel('Posición (°)')
ax.legend(ncol=2, fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[0, 1]
for i in range(6):
    ax.plot(t_full, np.degrees(dq_full[:, i]), color=COLORS[i])
for wt in wp_times[1:-1]:
    ax.axvline(x=wt, color='gray', linestyle='--', alpha=0.5)
ax.set_title('b) Velocidad articular dq(t)')
ax.set_ylabel('Velocidad (°/s)')
ax.grid(True, alpha=0.3)

ax = axes[1, 0]
for i in range(6):
    ax.plot(t_full, u_full[:, i], color=COLORS[i])
for wt in wp_times[1:-1]:
    ax.axvline(x=wt, color='gray', linestyle='--', alpha=0.5)
ax.set_title('c) Control óptimo u*(t) = jerk')
ax.set_ylabel('Jerk (rad/s³)')
ax.set_xlabel('Tiempo (s)')
ax.grid(True, alpha=0.3)

ax = axes[1, 1]
ax.bar(['Seg 1\nHome→T1', 'Seg 2\nT1→T2', 'Seg 3\nT2→Home'], J_segments, color=['steelblue', 'coral', 'seagreen'])
ax.set_title(f'd) Costo por segmento (Total: {sum(J_segments):.4f})')
ax.set_ylabel('Costo J')
ax.grid(True, alpha=0.3, axis='y')

for i, v in enumerate(J_segments):
    ax.text(i, v + 0.001, f'{v:.4f}', ha='center', fontsize=10)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, '04_multi_waypoint.png'), dpi=150, bbox_inches='tight')
plt.close()

print(f"  Costos por segmento: {[f'{j:.4f}' for j in J_segments]}")
print(f"  Costo total: {sum(J_segments):.4f}")

# ============================================================
# Experimento 5: Comparación PMP vs Trapezoidal
# ============================================================
print("\n=== Experimento 5: PMP vs Perfil Trapezoidal ===")

T = 3.0
pmp_res = plan_pmp_minimum_jerk(q0, zeros6, zeros6, q1, zeros6, zeros6, T)
trap_res = trapezoidal_velocity_profile(q0, q1, T, dt=0.001)

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Experimento 5: PMP Minimum-Jerk vs Perfil Trapezoidal\n(Junta 1 — Base, T=3s)', fontsize=14, fontweight='bold')

ji = 0  # Joint index to display

ax = axes[0, 0]
ax.plot(pmp_res['t'], np.degrees(pmp_res['q'][:, ji]), 'b-', label='PMP', linewidth=2)
ax.plot(trap_res['t'], np.degrees(trap_res['q'][:, ji]), 'r--', label='Trapezoidal', linewidth=2)
ax.set_title('a) Posición q₁(t)')
ax.set_ylabel('Posición (°)')
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[0, 1]
ax.plot(pmp_res['t'], np.degrees(pmp_res['dq'][:, ji]), 'b-', label='PMP', linewidth=2)
ax.plot(trap_res['t'], np.degrees(trap_res['dq'][:, ji]), 'r--', label='Trapezoidal', linewidth=2)
ax.set_title('b) Velocidad dq₁(t)')
ax.set_ylabel('Velocidad (°/s)')
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[1, 0]
ax.plot(pmp_res['t'], np.degrees(pmp_res['ddq'][:, ji]), 'b-', label='PMP', linewidth=2)
ax.plot(trap_res['t'], np.degrees(trap_res['ddq'][:, ji]), 'r--', label='Trapezoidal', linewidth=2)
ax.set_title('c) Aceleración ddq₁(t)')
ax.set_ylabel('Aceleración (°/s²)')
ax.set_xlabel('Tiempo (s)')
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[1, 1]
ax.plot(pmp_res['t'], pmp_res['J_acc'], 'b-', label=f'PMP (J={pmp_res["J_total"]:.4f})', linewidth=2)
ax.plot(trap_res['t'], trap_res['J_acc'], 'r--', label=f'Trapezoidal (J={trap_res["J_total"]:.4f})', linewidth=2)
ax.set_title('d) Costo acumulado J(t)')
ax.set_ylabel('J(t)')
ax.set_xlabel('Tiempo (s)')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, '05_pmp_vs_trapezoidal.png'), dpi=150, bbox_inches='tight')
plt.close()

ratio = trap_res['J_total'] / pmp_res['J_total']
print(f"  PMP J = {pmp_res['J_total']:.6f}")
print(f"  Trapezoidal J = {trap_res['J_total']:.6f}")
print(f"  Ratio (Trap/PMP) = {ratio:.2f}x")

# ============================================================
# Experimento extra: Tabla resumen de métricas
# ============================================================
print("\n=== Tabla resumen ===")

# Summary metrics for various T
summary_T = [1.0, 2.0, 3.0, 4.0, 5.0]
summary_data = []
for Tv in summary_T:
    r_pmp = plan_pmp_minimum_jerk(q0, zeros6, zeros6, q1, zeros6, zeros6, Tv)
    r_trap = trapezoidal_velocity_profile(q0, q1, Tv)
    summary_data.append({
        'T': Tv,
        'J_pmp': r_pmp['J_total'],
        'J_trap': r_trap['J_total'],
        'ratio': r_trap['J_total'] / max(r_pmp['J_total'], 1e-12),
        'max_vel_pmp': np.degrees(np.max(np.abs(r_pmp['dq']))),
        'max_jerk_pmp': np.max(np.abs(r_pmp['u'])),
    })

fig, ax = plt.subplots(figsize=(10, 3))
ax.axis('off')
table_data = [['T (s)', 'J_PMP', 'J_Trap', 'Ratio', 'Vel_max PMP (°/s)', 'Jerk_max PMP']]
for d in summary_data:
    table_data.append([
        f"{d['T']:.1f}",
        f"{d['J_pmp']:.4f}",
        f"{d['J_trap']:.4f}",
        f"{d['ratio']:.2f}x",
        f"{d['max_vel_pmp']:.2f}",
        f"{d['max_jerk_pmp']:.4f}",
    ])

table = ax.table(cellText=table_data[1:], colLabels=table_data[0],
                  loc='center', cellLoc='center')
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.2, 1.8)

# Header styling
for j in range(len(table_data[0])):
    table[0, j].set_facecolor('#4472C4')
    table[0, j].set_text_props(color='white', fontweight='bold')

# Alternate row colors
for i in range(1, len(table_data)):
    color = '#D6E4F0' if i % 2 == 0 else 'white'
    for j in range(len(table_data[0])):
        table[i, j].set_facecolor(color)

plt.title('Tabla Resumen: PMP vs Trapezoidal', fontsize=13, fontweight='bold', pad=20)
plt.savefig(os.path.join(OUTPUT_DIR, '06_summary_table.png'), dpi=150, bbox_inches='tight')
plt.close()

print("\n✓ Todos los experimentos completados. Gráficas guardadas en:", OUTPUT_DIR)
