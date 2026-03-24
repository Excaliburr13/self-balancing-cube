"""
Self-Balancing Cube — Full Realistic Simulation
================================================
Run:  python sim_full.py
Deps: numpy, matplotlib, scipy (already installed)

Models included:
  - RK4 rigid body physics
  - IMU gaussian noise + gyro bias drift
  - Motor deadband, saturation, torque lag
  - Battery voltage sag under load
  - Physical disturbance injection
  - Real-time 3D cube visualization
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from collections import deque

# ═══════════════════════════════════════════════════════════════
#  PHYSICAL CONSTANTS
# ═══════════════════════════════════════════════════════════════
g        = 9.81
M        = 0.250          # cube mass kg
L        = 0.075          # half-side m
I_cube   = (2/3)*M*L**2  # moment of inertia about edge
m_wheel  = 0.050          # reaction wheel mass kg
r_wheel  = 0.030          # wheel radius m
I_wheel  = 0.5*m_wheel*r_wheel**2

DT       = 0.001          # 1kHz control loop
HISTORY  = 600            # 6 seconds of history

# ═══════════════════════════════════════════════════════════════
#  IMU NOISE MODEL
# ═══════════════════════════════════════════════════════════════
class IMUModel:
    def __init__(self):
        self.gyro_noise_std  = 0.003    # rad/s white noise
        self.accel_noise_std = 0.015    # m/s² white noise
        self.gyro_bias       = 0.0      # current bias (rad/s)
        self.bias_drift_rate = 0.00002  # random walk rate
        self.alpha           = 0.98     # complementary filter
        self.angle_est       = 0.0      # filtered angle estimate
        self.enabled         = True

    def update(self, true_angle, true_rate, dt):
        if not self.enabled:
            return true_angle, true_rate

        # Gyro bias random walk
        self.gyro_bias += np.random.normal(0, self.bias_drift_rate)

        # Noisy measurements
        noisy_rate  = true_rate  + self.gyro_bias \
                    + np.random.normal(0, self.gyro_noise_std)
        noisy_accel = true_angle + np.random.normal(0, self.accel_noise_std)

        # Complementary filter
        self.angle_est = self.alpha * (self.angle_est + noisy_rate * dt) \
                       + (1 - self.alpha) * noisy_accel

        return self.angle_est, noisy_rate

    def reset(self):
        self.gyro_bias  = 0.0
        self.angle_est  = 0.0

# ═══════════════════════════════════════════════════════════════
#  BATTERY MODEL
# ═══════════════════════════════════════════════════════════════
class BatteryModel:
    def __init__(self):
        self.voltage_full    = 12.6     # 3S LiPo full V
        self.voltage_min     = 9.9      # 3S cutoff V
        self.voltage         = 12.6
        self.internal_resist = 0.08     # Ohms
        self.capacity_mah    = 1300.0
        self.charge_mah      = 1300.0
        self.enabled         = True

    @property
    def soc(self):
        return self.charge_mah / self.capacity_mah

    def update(self, current_draw_a, dt):
        if not self.enabled:
            return self.voltage_full

        # Voltage sag under load
        self.voltage = self.voltage_full * self.soc \
                     - self.internal_resist * current_draw_a

        # Capacity drain
        self.charge_mah -= (current_draw_a * dt / 3.6)
        self.charge_mah  = max(0, self.charge_mah)

        return max(self.voltage_min, self.voltage)

    def reset(self):
        self.charge_mah = self.capacity_mah
        self.voltage    = self.voltage_full

# ═══════════════════════════════════════════════════════════════
#  MOTOR MODEL
# ═══════════════════════════════════════════════════════════════
class MotorModel:
    def __init__(self):
        self.deadband        = 0.05     # N·m — min torque threshold
        self.max_torque      = 0.8      # N·m physical limit
        self.lag_tau         = 0.003    # s — electrical time constant
        self.actual_torque   = 0.0      # current output
        self.enabled         = True

    def update(self, demanded_torque, battery_voltage, dt):
        if not self.enabled:
            return 0.0

        # Battery scaling — less voltage = less max torque
        voltage_factor = battery_voltage / 12.6
        effective_max  = self.max_torque * voltage_factor

        # Deadband
        if abs(demanded_torque) < self.deadband:
            demanded_torque = 0.0

        # Saturation
        demanded_torque = np.clip(demanded_torque,
                                  -effective_max, effective_max)

        # First-order lag: τ·dT/dt + T = T_demand
        alpha = dt / (self.lag_tau + dt)
        self.actual_torque = alpha * demanded_torque \
                           + (1 - alpha) * self.actual_torque

        # Estimate current draw (rough: P = τ·ω, I = P/V)
        current = abs(self.actual_torque) * 50 / max(battery_voltage, 1)
        return self.actual_torque, current

    def reset(self):
        self.actual_torque = 0.0

# ═══════════════════════════════════════════════════════════════
#  PID CONTROLLER
# ═══════════════════════════════════════════════════════════════
class CubePID:
    def __init__(self):
        self.kp = 30.0
        self.ki = 0.25
        self.kd = 7.3
        self.integral      = 0.0
        self.prev_error    = 0.0
        self.prev_deriv    = 0.0
        self.deriv_alpha   = 0.05     # 50Hz cutoff at 1kHz
        self.integral_limit= 10.0
        self.out_max       = 0.8
        self.p = self.i = self.d = self.out = 0.0

    def compute(self, error, dt):
        # P
        self.p = self.kp * error

        # I with anti-windup
        self.integral = np.clip(self.integral + error * dt,
                                -self.integral_limit, self.integral_limit)
        self.i = self.ki * self.integral

        # D with low-pass filter
        raw_d      = (error - self.prev_error) / dt
        filt_d     = self.deriv_alpha * raw_d \
                   + (1 - self.deriv_alpha) * self.prev_deriv
        self.d     = self.kd * filt_d
        self.prev_deriv = filt_d
        self.prev_error = error

        self.out = np.clip(self.p + self.i + self.d,
                           -self.out_max, self.out_max)
        return self.out

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0
        self.prev_deriv = 0.0
        self.p = self.i = self.d = self.out = 0.0

# ═══════════════════════════════════════════════════════════════
#  PHYSICS ENGINE
# ═══════════════════════════════════════════════════════════════
class PhysicsEngine:
    def __init__(self):
        self.theta      = 0.08    # rad — initial tilt
        self.theta_dot  = 0.0
        self.omega_x    = 0.0    # wheel angular velocity
        self.omega_y    = 0.0

    def step(self, tau_x, tau_y, dt):
        def derivatives(state, tau):
            th, th_dot, om = state
            th_ddot = (M*g*L*np.sin(th) - tau) / (I_cube + I_wheel)
            om_dot  = tau / I_wheel
            return np.array([th_dot, th_ddot, om_dot])

        # RK4 for roll (X axis)
        s_x = np.array([self.theta, self.theta_dot, self.omega_x])
        k1 = derivatives(s_x, tau_x)
        k2 = derivatives(s_x + 0.5*dt*k1, tau_x)
        k3 = derivatives(s_x + 0.5*dt*k2, tau_x)
        k4 = derivatives(s_x + dt*k3, tau_x)
        s_x += (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

        self.theta     = s_x[0]
        self.theta_dot = s_x[1]
        self.omega_x   = s_x[2]

        return self.theta, self.theta_dot

    def apply_disturbance(self, magnitude=0.8):
        self.theta_dot += magnitude * (1 if np.random.rand()>0.5 else -1)

    def reset(self):
        self.theta     = 0.15
        self.theta_dot = 0.0
        self.omega_x   = 0.0
        self.omega_y   = 0.0

# ═══════════════════════════════════════════════════════════════
#  3D CUBE VISUALIZER
# ═══════════════════════════════════════════════════════════════
def make_cube_faces(theta):
    """Generate cube face vertices rotated by theta around X axis."""
    s = L
    verts = np.array([
        [-s,-s,-s], [ s,-s,-s], [ s, s,-s], [-s, s,-s],
        [-s,-s, s], [ s,-s, s], [ s, s, s], [-s, s, s]
    ])
    # Rotation around X axis
    Rx = np.array([
        [1,           0,            0],
        [0,  np.cos(theta), -np.sin(theta)],
        [0,  np.sin(theta),  np.cos(theta)]
    ])
    verts = (Rx @ verts.T).T

    faces = [
        [verts[0],verts[1],verts[2],verts[3]],
        [verts[4],verts[5],verts[6],verts[7]],
        [verts[0],verts[1],verts[5],verts[4]],
        [verts[2],verts[3],verts[7],verts[6]],
        [verts[0],verts[3],verts[7],verts[4]],
        [verts[1],verts[2],verts[6],verts[5]],
    ]
    return faces

# ═══════════════════════════════════════════════════════════════
#  MAIN SIMULATION GUI
# ═══════════════════════════════════════════════════════════════
class FullSimulator:
    def __init__(self):
        self.physics  = PhysicsEngine()
        self.imu      = IMUModel()
        self.battery  = BatteryModel()
        self.motor_x  = MotorModel()
        self.pid      = CubePID()

        self.running  = False
        self.t        = 0.0
        self.steps_per_frame = 50   # 10 physics steps per GUI frame

        # History buffers
        self.hist_t      = deque(maxlen=HISTORY)
        self.hist_theta  = deque(maxlen=HISTORY)
        self.hist_est    = deque(maxlen=HISTORY)
        self.hist_torque = deque(maxlen=HISTORY)
        self.hist_p      = deque(maxlen=HISTORY)
        self.hist_i      = deque(maxlen=HISTORY)
        self.hist_d      = deque(maxlen=HISTORY)
        self.hist_vbat   = deque(maxlen=HISTORY)

        self._build_gui()

    def _build_gui(self):
        self.fig = plt.figure(figsize=(16, 9), facecolor='#0d1117')
        self.fig.canvas.manager.set_window_title(
            'Self-Balancing Cube — Full Realistic Simulation')

        gs = gridspec.GridSpec(3, 3, figure=self.fig,
                               left=0.05, right=0.72,
                               top=0.93, bottom=0.08,
                               hspace=0.55, wspace=0.35)
        gs_3d = gridspec.GridSpec(1, 1, figure=self.fig,
                                   left=0.72, right=0.98,
                                   top=0.49, bottom=0.12)

        # ── Plot axes ─────────────────────────────────────────
        self.ax_angle  = self.fig.add_subplot(gs[0, :])
        self.ax_pid    = self.fig.add_subplot(gs[1, :])
        self.ax_bat    = self.fig.add_subplot(gs[2, :])
        self.ax_3d     = self.fig.add_subplot(gs_3d[0,0], projection='3d')

        for ax in [self.ax_angle, self.ax_pid, self.ax_bat]:
            ax.set_facecolor('#161b22')
            ax.tick_params(colors='#8b949e', labelsize=8)
            ax.set_xlabel('Time (s)', color='#8b949e', fontsize=8)
            for sp in ax.spines.values():
                sp.set_edgecolor('#30363d')
            ax.grid(True, color='#21262d', linewidth=0.5, linestyle='--')
            ax.axhline(0, color='#3fb950', lw=0.8, ls='--', alpha=0.5)

        self.ax_angle.set_title('Angle (rad) — true vs IMU estimate',
                                 color='#c9d1d9', fontsize=9)
        self.ax_pid.set_title('PID terms + torque',
                               color='#c9d1d9', fontsize=9)
        self.ax_bat.set_title('Battery voltage (V)',
                               color='#c9d1d9', fontsize=9)

        self.ax_3d.set_facecolor('#161b22')
        self.ax_3d.set_title('3D cube', color='#c9d1d9', fontsize=9, pad=2)
        self.ax_3d.tick_params(colors='#8b949e', labelsize=6)

        # ── Lines ─────────────────────────────────────────────
        self.ln_true, = self.ax_angle.plot([], [], '#58a6ff', lw=1.5,
                                            label='true angle')
        self.ln_est,  = self.ax_angle.plot([], [], '#f78166', lw=1,
                                            ls='--', label='IMU estimate')
        self.ln_p,    = self.ax_pid.plot([], [], '#58a6ff', lw=1, label='P')
        self.ln_i,    = self.ax_pid.plot([], [], '#3fb950', lw=1, label='I')
        self.ln_d,    = self.ax_pid.plot([], [], '#f78166', lw=1, label='D')
        self.ln_tau,  = self.ax_pid.plot([], [], '#ffa657', lw=2, label='τ')
        self.ln_vbat, = self.ax_bat.plot([], [], '#d2a8ff', lw=1.5,
                                          label='V_bat')

        self.ax_angle.legend(fontsize=7, facecolor='#161b22',
                              labelcolor='#c9d1d9', framealpha=0.8)
        self.ax_pid.legend(fontsize=7, facecolor='#161b22',
                            labelcolor='#c9d1d9', framealpha=0.8)

        # ── Status text ───────────────────────────────────────
        self.status_txt = self.fig.text(0.38, 0.97, 'STOPPED',
                                         color='#f85149', fontsize=11,
                                         fontweight='bold', ha='center')
        self.metrics_txt = self.fig.text(0.38, 0.945, '',
                                          color='#8b949e', fontsize=8,
                                          ha='center', fontfamily='monospace')

        # ── Right panel controls ───────────────────────────────
        sc = '#1c2333'
        def sl(ax_pos, label, vmin, vmax, vinit, color):
            ax = self.fig.add_axes(ax_pos, facecolor=sc)
            s  = Slider(ax, label, vmin, vmax, valinit=vinit, color=color)
            s.label.set_color('#c9d1d9')
            s.valtext.set_color('#c9d1d9')
            return s

        # PID sliders
        self.fig.text(0.745, 0.96, 'PID gains',
                      color='#c9d1d9', fontsize=10, fontweight='bold')
        self.sl_kp = sl([0.745,0.905,0.22,0.022],'Kp', 0,60,30,'#58a6ff')
        self.sl_ki = sl([0.745,0.875,0.22,0.022],'Ki', 0, 5,0.25,'#3fb950')
        self.sl_kd = sl([0.745,0.845,0.22,0.022],'Kd', 0, 13,7.3,'#f78166')

        # Realism sliders
        self.fig.text(0.745, 0.82, 'Realism',
                      color='#c9d1d9', fontsize=10, fontweight='bold')
        self.sl_noise   = sl([0.745,0.785,0.22,0.022],
                              'IMU noise',  0,5,  1,'#d2a8ff')
        self.sl_deadband= sl([0.745,0.755,0.22,0.022],
                              'Deadband',   0,0.3,0.05,'#ffa657')
        self.sl_sag     = sl([0.745,0.725,0.22,0.022],
                              'Batt sag',   0,1,  0.5,'#f78166')
        self.sl_dist    = sl([0.745,0.695,0.22,0.022],
                              'Push mag',   0,3,  0.8,'#58a6ff')

        # Buttons
        def btn(pos, label, fc, hc):
            ax = self.fig.add_axes(pos)
            b  = Button(ax, label, color=fc, hovercolor=hc)
            b.label.set_color('white')
            return b

        self.btn_start = btn([0.745,0.635,0.10,0.04],
                              'Start','#238636','#2ea043')
        self.btn_stop  = btn([0.855,0.635,0.10,0.04],
                              'Stop', '#da3633','#f85149')
        self.btn_reset = btn([0.745,0.585,0.10,0.04],
                              'Reset','#30363d','#484f58')
        self.btn_push  = btn([0.855,0.585,0.10,0.04],
                              'Push!','#9a6700','#b07800')

        # Toggle checkboxes (simulated with buttons)
        self.imu_on  = True
        self.bat_on  = True
        self.mot_on  = True

        self.btn_imu = btn([0.745,0.530,0.06,0.035],
                            'IMU','#0f6e56','#1D9E75')
        self.btn_bat = btn([0.815,0.530,0.06,0.035],
                            'BAT','#0f6e56','#1D9E75')
        self.btn_mot = btn([0.885,0.530,0.06,0.035],
                            'MOT','#0f6e56','#1D9E75')

        self.fig.text(0.745, 0.57, 'Toggle models (green=on):',
                      color='#8b949e', fontsize=8)

        # Wire callbacks
        self.btn_start.on_clicked(lambda e: self._start())
        self.btn_stop.on_clicked(lambda e:  self._stop())
        self.btn_reset.on_clicked(lambda e: self._reset())
        self.btn_push.on_clicked(lambda e:  self._push())
        self.btn_imu.on_clicked(self._toggle_imu)
        self.btn_bat.on_clicked(self._toggle_bat)
        self.btn_mot.on_clicked(self._toggle_mot)

        # Slider callbacks
        for s in [self.sl_kp, self.sl_ki, self.sl_kd]:
            s.on_changed(self._update_gains)
        self.sl_noise.on_changed(self._update_noise)
        self.sl_deadband.on_changed(self._update_deadband)
        self.sl_sag.on_changed(self._update_sag)

    # ── Callbacks ─────────────────────────────────────────────
    def _start(self):
        self.running = True
        self.status_txt.set_text('RUNNING')
        self.status_txt.set_color('#3fb950')

    def _stop(self):
        self.running = False
        self.status_txt.set_text('PAUSED')
        self.status_txt.set_color('#ffa657')

    def _reset(self):
        self.running = False
        self.t = 0.0
        self.physics.reset()
        self.imu.reset()
        self.battery.reset()
        self.motor_x.reset()
        self.pid.reset()
        for h in [self.hist_t, self.hist_theta, self.hist_est,
                  self.hist_torque, self.hist_p, self.hist_i,
                  self.hist_d, self.hist_vbat]:
            h.clear()
        self.status_txt.set_text('RESET')
        self.status_txt.set_color('#f85149')

    def _push(self):
        mag = self.sl_dist.val
        self.physics.apply_disturbance(mag)

    def _update_gains(self, _):
        self.pid.kp = self.sl_kp.val
        self.pid.ki = self.sl_ki.val
        self.pid.kd = self.sl_kd.val

    def _update_noise(self, v):
        self.imu.gyro_noise_std  = 0.003 * v
        self.imu.accel_noise_std = 0.015 * v

    def _update_deadband(self, v):
        self.motor_x.deadband = v

    def _update_sag(self, v):
        self.battery.internal_resist = 0.08 * v * 2

    def _toggle_imu(self, e):
        self.imu_on = not self.imu_on
        self.imu.enabled = self.imu_on
        self.btn_imu.color = '#0f6e56' if self.imu_on else '#30363d'

    def _toggle_bat(self, e):
        self.bat_on = not self.bat_on
        self.battery.enabled = self.bat_on
        self.btn_bat.color = '#0f6e56' if self.bat_on else '#30363d'

    def _toggle_mot(self, e):
        self.mot_on = not self.mot_on
        self.motor_x.enabled = self.mot_on
        self.btn_mot.color = '#0f6e56' if self.mot_on else '#30363d'

    # ── Physics step ──────────────────────────────────────────
    def _step(self):
        angle_est = self.physics.theta  # default fallback
        rate_est  = self.physics.theta_dot
        for _ in range(self.steps_per_frame):
            if not self.running:
                break

            # Safety cutoff
            if abs(self.physics.theta) > 0.6:
                self.running = False
                self.status_txt.set_text('FELL OVER')
                self.status_txt.set_color('#f85149')
                break

            # IMU measurement
            angle_est, rate_est = self.imu.update(
                self.physics.theta, self.physics.theta_dot, DT)

            # PID
            error  = 0.0 - angle_est
            tau_d  = -self.pid.compute(error, DT)

            # Battery
            vbat = self.battery.update(abs(tau_d)*3, DT)

            # Motor
            result = self.motor_x.update(tau_d, vbat, DT)
            if isinstance(result, tuple):
                tau_actual, _ = result
            else:
                tau_actual = result

            # Physics
            print(f"tau_actual={tau_actual:.6f}  tau_d={tau_d:.6f}  running={self.running}")
            theta, _ = self.physics.step(tau_actual, 0.0, DT)

            self.t += DT

        # Record history
        self.hist_t.append(self.t)
        self.hist_theta.append(self.physics.theta)
        self.hist_est.append(angle_est if self.running or len(self.hist_est)==0
                             else self.hist_est[-1])
        self.hist_torque.append(self.pid.out)
        self.hist_p.append(self.pid.p)
        self.hist_i.append(self.pid.i)
        self.hist_d.append(self.pid.d)
        self.hist_vbat.append(self.battery.voltage)

    # ── 3D cube update ────────────────────────────────────────
    def _update_3d(self, theta):
        self.ax_3d.cla()
        self.ax_3d.set_facecolor('#161b22')
        self.ax_3d.set_xlim([-0.12, 0.12])
        self.ax_3d.set_ylim([-0.12, 0.12])
        self.ax_3d.set_zlim([-0.12, 0.12])
        self.ax_3d.set_title(f'θ = {np.degrees(theta):.1f}°',
                              color='#c9d1d9', fontsize=9, pad=2)
        self.ax_3d.tick_params(colors='#30363d', labelsize=0)
        self.ax_3d.set_xlabel('', color='#30363d')
        self.ax_3d.set_ylabel('', color='#30363d')
        self.ax_3d.set_zlabel('', color='#30363d')

        # Cube faces
        faces = make_cube_faces(theta)
        stable = abs(theta) < 0.05
        face_color = '#1D9E75' if stable else '#D85A30'
        alpha = 0.55

        poly = Poly3DCollection(faces,
                         facecolor=face_color,
                         edgecolor='#58a6ff',
                         linewidth=0.8)
        poly.set_alpha(0.55)


        self.ax_3d.add_collection3d(poly)

        # Balance point indicator — vertical line
        self.ax_3d.plot([0,0],[0,0],[-0.12,0.12],
                         color='#3fb950', lw=0.8, ls='--', alpha=0.5)

        # Wheel indicator (circle on X face)
        phi = np.linspace(0, 2*np.pi, 30)
        wx  = np.full(30, L)
        wy  = r_wheel * np.cos(phi)
        wz  = r_wheel * np.sin(phi)
        Rx  = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],
                         [0,np.sin(theta), np.cos(theta)]])
        w_pts = (Rx @ np.vstack([wx,wy,wz])).T
        self.ax_3d.plot(w_pts[:,0], w_pts[:,1], w_pts[:,2],
                         color='#ffa657', lw=1.5)

    # ── Animation frame ───────────────────────────────────────
    def _animate(self, frame):
        self._step()

        t_arr = list(self.hist_t)
        if not t_arr:
            return []

        def update_line(line, xdata, ydata, ax):
            line.set_data(xdata, ydata)
            ax.relim(); ax.autoscale_view()
            if t_arr[-1] > t_arr[0]:
                ax.set_xlim(t_arr[0], t_arr[-1])

        update_line(self.ln_true,  t_arr, list(self.hist_theta),  self.ax_angle)
        update_line(self.ln_est,   t_arr, list(self.hist_est),    self.ax_angle)
        update_line(self.ln_p,     t_arr, list(self.hist_p),      self.ax_pid)
        update_line(self.ln_i,     t_arr, list(self.hist_i),      self.ax_pid)
        update_line(self.ln_d,     t_arr, list(self.hist_d),      self.ax_pid)
        update_line(self.ln_tau,   t_arr, list(self.hist_torque), self.ax_pid)
        # Fix ax_pid minimum readable Y range
        ymin, ymax = self.ax_pid.get_ylim()
        if abs(ymax - ymin) < 0.01:
            self.ax_pid.set_ylim(-0.05, 0.05)

        update_line(self.ln_vbat,  t_arr, list(self.hist_vbat),  self.ax_bat)

        # 3D cube
        self._update_3d(self.physics.theta)

        # Metrics
        if self.hist_theta:
            th = np.degrees(self.physics.theta)
            vb = self.battery.voltage
            lp = abs(self.pid.out)
            ss = abs(self.hist_theta[-1]) < 0.01
            self.metrics_txt.set_text(
                f"θ={th:.2f}°  V={vb:.2f}V  τ={lp:.3f}N·m  "
                f"t={self.t:.1f}s  {'STABLE' if ss else 'balancing...'}"
            )

        return (self.ln_true, self.ln_est, self.ln_p, self.ln_i,
                self.ln_d, self.ln_tau, self.ln_vbat)

    def run(self):
        self.fig.suptitle('Self-Balancing Cube — Full Realistic Simulation',
                           color='#e6edf3', fontsize=12,
                           fontweight='bold', y=0.995)
        self.ani = animation.FuncAnimation(
            self.fig, self._animate,
            interval=33, blit=False, cache_frame_data=False)
        plt.show()

# ═══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("=" * 55)
    print("  Self-Balancing Cube — Full Realistic Simulation")
    print("=" * 55)
    print("  Controls:")
    print("    Start — begin simulation")
    print("    Stop  — pause")
    print("    Reset — restart from initial tilt")
    print("    Push! — apply random disturbance")
    print("    IMU/BAT/MOT — toggle realism models on/off")
    print("=" * 55)
    sim = FullSimulator()
    sim.run()
