"""
Self-Balancing Cube — Live Serial Plotter
==========================================
Run: python plotter.py
Reads CSV telemetry from firmware over USB serial
Plots roll, pitch, PID terms, and torque in real time

Usage:
  1. Flash firmware to Nucleo
  2. Connect USB cable
  3. Run this script
  4. Type ARM in the serial monitor panel to start balancing
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
from collections import deque
import threading
import time
import sys

# ─── Config ────────────────────────────────────────────────────
BAUD_RATE    = 115200
WINDOW_SIZE  = 500       # Number of data points to display
UPDATE_MS    = 50        # Plot refresh rate (ms)
CSV_HEADER   = "t_ms,roll_deg,pitch_deg,P,I,D,tau_x,tau_y,armed"

# ─── Data buffers ──────────────────────────────────────────────
t        = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
roll     = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
pitch    = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
p_term   = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
i_term   = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
d_term   = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
tau_x    = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
armed_st = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)

data_lock  = threading.Lock()
ser        = None
running    = True
last_line  = ""
parse_errors = 0

# ─── Auto-detect serial port ───────────────────────────────────
def find_nucleo_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").upper()
        if "STM32" in desc or "ST-LINK" in desc or "NUCLEO" in desc \
           or "USB SERIAL" in desc or "COM" in p.device:
            print(f"[PORT] Found: {p.device} — {p.description}")
            return p.device
    # Fallback: show all ports and ask user
    print("[PORT] Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    if not ports:
        print("[PORT] No serial ports found — is the Nucleo connected?")
        return None
    idx = input("Select port number: ")
    try:
        return ports[int(idx)].device
    except:
        return None

# ─── Serial reader thread ──────────────────────────────────────
def serial_reader(port):
    global ser, running, last_line, parse_errors
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"[SERIAL] Connected to {port} at {BAUD_RATE} baud")
    except Exception as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        running = False
        return

    while running:
        try:
            raw = ser.readline().decode('utf-8', errors='ignore').strip()
            if not raw or raw.startswith('[') or raw.startswith('=') \
               or raw.startswith('-') or raw.startswith('>'):
                # System messages — print to console, skip parsing
                if raw:
                    print(f"[FW] {raw}")
                continue

            if raw == CSV_HEADER:
                continue    # Skip header line

            parts = raw.split(',')
            if len(parts) != 9:
                continue

            with data_lock:
                t.append(float(parts[0]) / 1000.0)   # ms → s
                roll.append(float(parts[1]))
                pitch.append(float(parts[2]))
                p_term.append(float(parts[3]))
                i_term.append(float(parts[4]))
                d_term.append(float(parts[5]))
                tau_x.append(float(parts[6]))
                armed_st.append(float(parts[8]))
                last_line = raw

        except ValueError:
            parse_errors += 1
        except Exception as e:
            if running:
                print(f"[SERIAL] Error: {e}")
            break

    if ser and ser.is_open:
        ser.close()

# ─── Send command to firmware ──────────────────────────────────
def send_command(cmd):
    if ser and ser.is_open:
        ser.write((cmd + '\n').encode())
        print(f"[CMD] Sent: {cmd}")

# ─── Build GUI ─────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 9), facecolor='#0d1117')
fig.canvas.manager.set_window_title('Self-Balancing Cube — Live Telemetry')

gs = gridspec.GridSpec(3, 2, figure=fig,
                       left=0.07, right=0.97,
                       top=0.92, bottom=0.10,
                       hspace=0.5, wspace=0.3)

ax_roll   = fig.add_subplot(gs[0, 0])
ax_pitch  = fig.add_subplot(gs[0, 1])
ax_pid    = fig.add_subplot(gs[1, :])
ax_torque = fig.add_subplot(gs[2, :])

axes = [ax_roll, ax_pitch, ax_pid, ax_torque]
for ax in axes:
    ax.set_facecolor('#161b22')
    ax.tick_params(colors='#8b949e', labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor('#30363d')
    ax.grid(True, color='#21262d', linewidth=0.5, linestyle='--')

ax_roll.set_title('Roll angle (°)',   color='#c9d1d9', fontsize=9)
ax_pitch.set_title('Pitch angle (°)', color='#c9d1d9', fontsize=9)
ax_pid.set_title('PID terms',         color='#c9d1d9', fontsize=9)
ax_torque.set_title('Torque X (N·m)', color='#c9d1d9', fontsize=9)

for ax in axes:
    ax.set_xlabel('Time (s)', color='#8b949e', fontsize=8)
    ax.axhline(0, color='#3fb950', linewidth=0.8, linestyle='--', alpha=0.5)

line_roll,   = ax_roll.plot([], [],  color='#58a6ff', lw=1.5, label='roll')
line_pitch,  = ax_pitch.plot([], [], color='#d2a8ff', lw=1.5, label='pitch')
line_p,      = ax_pid.plot([], [],   color='#58a6ff', lw=1.2, label='P')
line_i,      = ax_pid.plot([], [],   color='#3fb950', lw=1.2, label='I')
line_d,      = ax_pid.plot([], [],   color='#f78166', lw=1.2, label='D')
line_torque, = ax_torque.plot([], [], color='#ffa657', lw=1.5, label='τ_x')

ax_pid.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', framealpha=0.8)

# Status text
status_text = fig.text(0.5, 0.96, 'DISARMED — connecting...',
                        ha='center', color='#f85149',
                        fontsize=11, fontweight='bold')
info_text = fig.text(0.5, 0.93, '',
                      ha='center', color='#8b949e', fontsize=8)

# ─── Command buttons ───────────────────────────────────────────
from matplotlib.widgets import Button

btn_ax_arm    = fig.add_axes([0.10, 0.02, 0.10, 0.04])
btn_ax_disarm = fig.add_axes([0.22, 0.02, 0.10, 0.04])
btn_ax_reset  = fig.add_axes([0.34, 0.02, 0.10, 0.04])

btn_arm    = Button(btn_ax_arm,    'ARM',    color='#238636', hovercolor='#2ea043')
btn_disarm = Button(btn_ax_disarm, 'DISARM', color='#da3633', hovercolor='#f85149')
btn_reset  = Button(btn_ax_reset,  'RESET',  color='#30363d', hovercolor='#484f58')

btn_arm.label.set_color('white')
btn_disarm.label.set_color('white')
btn_reset.label.set_color('#c9d1d9')

btn_arm.on_clicked(lambda e: send_command('ARM'))
btn_disarm.on_clicked(lambda e: send_command('DISARM'))
btn_reset.on_clicked(lambda e: send_command('RESET'))

# ─── Animation update ──────────────────────────────────────────
def animate(frame):
    with data_lock:
        t_data     = list(t)
        roll_data  = list(roll)
        pitch_data = list(pitch)
        p_data     = list(p_term)
        i_data     = list(i_term)
        d_data     = list(d_term)
        tau_data   = list(tau_x)
        arm_data   = list(armed_st)

    line_roll.set_data(t_data, roll_data)
    line_pitch.set_data(t_data, pitch_data)
    line_p.set_data(t_data, p_data)
    line_i.set_data(t_data, i_data)
    line_d.set_data(t_data, d_data)
    line_torque.set_data(t_data, tau_data)

    for ax, lines in [
        (ax_roll,   [line_roll]),
        (ax_pitch,  [line_pitch]),
        (ax_pid,    [line_p, line_i, line_d]),
        (ax_torque, [line_torque])
    ]:
        ax.relim(); ax.autoscale_view()
        if t_data[-1] > t_data[0]:
            ax.set_xlim(t_data[0], t_data[-1])

    # Status indicator
    is_armed = arm_data[-1] > 0.5
    status_text.set_text('ARMED — balancing active' if is_armed else 'DISARMED')
    status_text.set_color('#3fb950' if is_armed else '#f85149')

    # Info line
    if roll_data:
        info_text.set_text(
            f"roll={roll_data[-1]:.2f}°  pitch={pitch_data[-1]:.2f}°  "
            f"τ={tau_data[-1]:.3f}N·m  parse_errors={parse_errors}"
        )

    return (line_roll, line_pitch, line_p, line_i, line_d,
            line_torque, status_text, info_text)

# ─── Entry point ───────────────────────────────────────────────
if __name__ == '__main__':
    port = find_nucleo_port()
    if port is None:
        print("[ERROR] No port selected — exiting")
        sys.exit(1)

    thread = threading.Thread(target=serial_reader, args=(port,), daemon=True)
    thread.start()

    ani = animation.FuncAnimation(fig, animate, interval=UPDATE_MS,
                                   blit=False, cache_frame_data=False)

    plt.suptitle('Self-Balancing Cube — Live Telemetry',
                  color='#e6edf3', fontsize=12, fontweight='bold', y=0.99)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        print("\n[PLOTTER] Closed")
