"""
Self-Balancing Cube — Physics Simulation + PID Tuner
=====================================================
Run:  python simulator.py
Deps: numpy, matplotlib, scipy  (all already installed)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches
from matplotlib.widgets import Slider, Button, RadioButtons
from scipy.integrate import solve_ivp

# ─── Physical constants ────────────────────────────────────────────────────────
g   = 9.81          # gravity m/s²
M   = 0.250         # cube mass kg
L   = 0.075         # half side length m  (15cm cube)
I_c = (2/3)*M*L**2  # cube moment of inertia about edge (thin slab approx)
m_w = 0.050         # reaction wheel mass kg
r_w = 0.030         # wheel radius m
I_w = 0.5*m_w*r_w**2  # wheel moment of inertia

# ─── Default PID gains ─────────────────────────────────────────────────────────
DEFAULT_KP = 18.0
DEFAULT_KI = 0.8
DEFAULT_KD = 1.4

# ─── Simulation parameters ─────────────────────────────────────────────────────
DT      = 0.001     # 1 kHz control loop
T_END   = 8.0       # seconds
THETA0  = 0.18      # initial tilt angle (rad) ~10 degrees

# ─── PID Controller ────────────────────────────────────────────────────────────
class PIDController:
    def __init__(self, kp, ki, kd, dt, u_max=5.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.u_max = u_max
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        # Anti-windup clamp
        self.integral = np.clip(self.integral, -self.u_max/self.ki if self.ki else -1e6,
                                               self.u_max/self.ki if self.ki else 1e6)
        derivative = (error - self.prev_error) / self.dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        return np.clip(output, -self.u_max, self.u_max)


# ─── Plant model: cube on edge with reaction wheel ─────────────────────────────
def plant_odes(t, state, controller, disturbance_fn):
    theta, theta_dot, omega_w = state
    error = 0.0 - theta          # setpoint = 0 rad (balanced)
    tau   = controller.compute(error)

    # External disturbance torque
    tau_dist = disturbance_fn(t)

    # Equations of motion
    # Cube: (I_c + I_w)*theta_ddot = M*g*L*sin(theta) - tau + tau_dist
    # Wheel: I_w*omega_w_dot = tau
    theta_ddot = (M*g*L*np.sin(theta) - tau + tau_dist) / (I_c + I_w)
    omega_w_dot = tau / I_w

    return [theta_dot, theta_ddot, omega_w_dot]


# ─── Run simulation ────────────────────────────────────────────────────────────
def run_simulation(kp, ki, kd, disturbance_type='none', noise_std=0.0):
    pid = PIDController(kp, ki, kd, DT)
    t_span = (0, T_END)
    t_eval = np.arange(0, T_END, DT)
    state0 = [THETA0, 0.0, 0.0]

    # Disturbance profiles
    def disturbance_fn(t):
        if disturbance_type == 'impulse':
            return 0.3 if 2.0 < t < 2.05 else 0.0
        elif disturbance_type == 'step':
            return 0.15 if t > 3.0 else 0.0
        elif disturbance_type == 'sine':
            return 0.1 * np.sin(2*np.pi*1.5*t)
        return 0.0

    sol = solve_ivp(
        lambda t, y: plant_odes(t, y, pid, disturbance_fn),
        t_span, state0, t_eval=t_eval,
        method='RK45', max_step=DT*2, rtol=1e-6
    )

    theta   = sol.y[0]
    theta_d = sol.y[1]
    omega_w = sol.y[2]
    t       = sol.t

    # Recompute control output for plotting
    pid2 = PIDController(kp, ki, kd, DT)
    ctrl_out = np.array([pid2.compute(-th) for th in theta])

    # Add sensor noise
    if noise_std > 0:
        theta = theta + np.random.normal(0, noise_std, len(theta))

    return t, theta, theta_d, omega_w, ctrl_out


# ─── GUI ───────────────────────────────────────────────────────────────────────
class SimulatorGUI:
    def __init__(self):
        self.fig = plt.figure(figsize=(15, 9), facecolor='#0d1117')
        self.fig.canvas.manager.set_window_title('Self-Balancing Cube — PID Simulator')

        gs = gridspec.GridSpec(3, 3, figure=self.fig,
                               left=0.06, right=0.72,
                               top=0.93, bottom=0.08,
                               hspace=0.45, wspace=0.35)

        # Plot axes
        self.ax_theta  = self.fig.add_subplot(gs[0, :])
        self.ax_omega  = self.fig.add_subplot(gs[1, :])
        self.ax_ctrl   = self.fig.add_subplot(gs[2, :])

        self._style_axes()

        # ── Sliders ──────────────────────────────────────────────────────────
        sl_color   = '#1c2333'
        sl_ax_kp   = self.fig.add_axes([0.76, 0.80, 0.20, 0.025], facecolor=sl_color)
        sl_ax_ki   = self.fig.add_axes([0.76, 0.74, 0.20, 0.025], facecolor=sl_color)
        sl_ax_kd   = self.fig.add_axes([0.76, 0.68, 0.20, 0.025], facecolor=sl_color)
        sl_ax_ns   = self.fig.add_axes([0.76, 0.62, 0.20, 0.025], facecolor=sl_color)

        self.sl_kp = Slider(sl_ax_kp, 'Kp', 0.0, 60.0, valinit=DEFAULT_KP, color='#58a6ff')
        self.sl_ki = Slider(sl_ax_ki, 'Ki', 0.0,  5.0, valinit=DEFAULT_KI, color='#3fb950')
        self.sl_kd = Slider(sl_ax_kd, 'Kd', 0.0,  8.0, valinit=DEFAULT_KD, color='#f78166')
        self.sl_ns = Slider(sl_ax_ns, 'Noise σ', 0.0, 0.05, valinit=0.0,   color='#d2a8ff')

        for sl in [self.sl_kp, self.sl_ki, self.sl_kd, self.sl_ns]:
            sl.label.set_color('#c9d1d9')
            sl.valtext.set_color('#c9d1d9')

        # ── Disturbance selector ──────────────────────────────────────────────
        radio_ax = self.fig.add_axes([0.76, 0.42, 0.20, 0.16], facecolor='#0d1117')
        self.radio = RadioButtons(radio_ax,
                                  ('None', 'Impulse', 'Step', 'Sine wave'),
                                  activecolor='#58a6ff')
        radio_ax.set_title('Disturbance', color='#8b949e', fontsize=9, pad=4)
        for label in self.radio.labels:
            label.set_color('#c9d1d9')
            label.set_fontsize(9)

        # ── Buttons ───────────────────────────────────────────────────────────
        btn_ax_run   = self.fig.add_axes([0.76, 0.32, 0.09, 0.045])
        btn_ax_reset = self.fig.add_axes([0.87, 0.32, 0.09, 0.045])
        self.btn_run   = Button(btn_ax_run,   'Run',   color='#238636', hovercolor='#2ea043')
        self.btn_reset = Button(btn_ax_reset, 'Reset', color='#30363d', hovercolor='#484f58')
        self.btn_run.label.set_color('white')
        self.btn_reset.label.set_color('#c9d1d9')

        # ── Metrics text ──────────────────────────────────────────────────────
        self.metrics_ax = self.fig.add_axes([0.76, 0.08, 0.20, 0.20], facecolor='#161b22')
        self.metrics_ax.set_xticks([]); self.metrics_ax.set_yticks([])
        for spine in self.metrics_ax.spines.values():
            spine.set_edgecolor('#30363d')
        self.metrics_text = self.metrics_ax.text(
            0.05, 0.92, 'Run simulation to see metrics',
            transform=self.metrics_ax.transAxes,
            color='#8b949e', fontsize=8.5, va='top', fontfamily='monospace'
        )
        self.metrics_ax.set_title('Performance metrics', color='#8b949e', fontsize=8, pad=4)

        # ── Panel title ───────────────────────────────────────────────────────
        self.fig.text(0.76, 0.97, 'PID Tuning', color='#c9d1d9',
                      fontsize=11, fontweight='bold', va='top')
        self.fig.text(0.76, 0.93, f'Plant: M={M}kg  L={L*100:.0f}cm  Iw={I_w*1e6:.1f}g·cm²',
                      color='#8b949e', fontsize=7.5, va='top')

        # ── Wire up callbacks ─────────────────────────────────────────────────
        self.btn_run.on_clicked(self._on_run)
        self.btn_reset.on_clicked(self._on_reset)

        # Store line handles
        self.lines = {}
        self._init_plots()

        # Run once with defaults
        self._on_run(None)

    def _style_axes(self):
        titles = ['Tilt angle θ (rad)', 'Wheel speed ω (rad/s)', 'Control torque τ (N·m)']
        ylabels = ['θ (rad)', 'ω (rad/s)', 'τ (N·m)']
        refs    = [0.0, None, 0.0]
        axes    = [self.ax_theta, self.ax_omega, self.ax_ctrl]

        for ax, title, ylabel, ref in zip(axes, titles, ylabels, refs):
            ax.set_facecolor('#161b22')
            ax.tick_params(colors='#8b949e', labelsize=8)
            ax.set_title(title, color='#c9d1d9', fontsize=9, pad=4)
            ax.set_ylabel(ylabel, color='#8b949e', fontsize=8)
            ax.set_xlabel('Time (s)', color='#8b949e', fontsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor('#30363d')
            ax.grid(True, color='#21262d', linewidth=0.5, linestyle='--')
            if ref is not None:
                ax.axhline(ref, color='#3fb950', linewidth=0.8,
                           linestyle='--', alpha=0.6, label='Setpoint')

    def _init_plots(self):
        t_dummy = np.linspace(0, T_END, 100)
        z = np.zeros(100)
        self.lines['theta'], = self.ax_theta.plot(t_dummy, z, color='#58a6ff', lw=1.2, label='θ')
        self.lines['omega'], = self.ax_omega.plot(t_dummy, z, color='#d2a8ff', lw=1.2, label='ω wheel')
        self.lines['ctrl'],  = self.ax_ctrl.plot( t_dummy, z, color='#f78166', lw=1.2, label='τ')
        self.ax_theta.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', framealpha=0.8)
        self.ax_omega.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', framealpha=0.8)
        self.ax_ctrl.legend( fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', framealpha=0.8)

    def _disturbance_type(self):
        mapping = {
            'None': 'none', 'Impulse': 'impulse',
            'Step': 'step', 'Sine wave': 'sine'
        }
        return mapping[self.radio.value_selected]

    def _on_run(self, event):
        kp = self.sl_kp.val
        ki = self.sl_ki.val
        kd = self.sl_kd.val
        ns = self.sl_ns.val
        dist = self._disturbance_type()

        t, theta, theta_d, omega_w, ctrl = run_simulation(kp, ki, kd, dist, ns)

        self.lines['theta'].set_data(t, theta)
        self.lines['omega'].set_data(t, omega_w)
        self.lines['ctrl'].set_data(t, ctrl)

        for ax in [self.ax_theta, self.ax_omega, self.ax_ctrl]:
            ax.relim(); ax.autoscale_view()
            ax.set_xlim(0, T_END)

        # Metrics
        settled = np.where(np.abs(theta) < 0.01)[0]
        settle_t = t[settled[0]] if len(settled) > 0 else float('nan')
        overshoot = (np.max(np.abs(theta)) - abs(THETA0)) / abs(THETA0) * 100
        ss_error  = np.mean(np.abs(theta[-200:]))
        max_omega = np.max(np.abs(omega_w))
        max_tau   = np.max(np.abs(ctrl))
        stable    = ss_error < 0.005

        status = '✓ STABLE' if stable else '✗ UNSTABLE'
        metrics_str = (
            f"Kp={kp:.1f}  Ki={ki:.2f}  Kd={kd:.2f}\n"
            f"─────────────────────\n"
            f"Status     : {status}\n"
            f"Settle time: {settle_t:.3f} s\n"
            f"Overshoot  : {max(0,overshoot):.1f} %\n"
            f"SS error   : {ss_error*1000:.2f} mrad\n"
            f"Max ω wheel: {max_omega:.1f} rad/s\n"
            f"Max torque : {max_tau:.3f} N·m\n"
            f"Disturbance: {dist}"
        )
        color = '#3fb950' if stable else '#f85149'
        self.metrics_text.set_text(metrics_str)
        self.metrics_text.set_color(color)
        self.fig.canvas.draw_idle()

    def _on_reset(self, event):
        self.sl_kp.reset()
        self.sl_ki.reset()
        self.sl_kd.reset()
        self.sl_ns.reset()
        self._on_run(None)

    def show(self):
        self.fig.suptitle(
            'Self-Balancing Cube — PID Simulation',
            color='#e6edf3', fontsize=13, fontweight='bold', y=0.99
        )
        plt.show()


# ─── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 52)
    print("  Self-Balancing Cube — PID Simulator")
    print(f"  Plant: M={M}kg, L={L*100}cm, Iw={I_w*1e6:.2f} g·cm²")
    print(f"  Loop:  dt={DT*1000}ms  ({int(1/DT)} Hz)")
    print(f"  Init:  θ₀={np.degrees(THETA0):.1f}°")
    print("=" * 52)
    gui = SimulatorGUI()
    gui.show()
