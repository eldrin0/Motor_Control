import serial
import time
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, TextBox, Button, RadioButtons

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
DT = 0.02          

# Define individual PPR for each motor here
MOTOR_PPR = {
    1: 12.0,
    2: 12.0,
    3: 12.0,
    4: 630.0
}

# --- Graph Data Storage ---
HISTORY_LENGTH = 1000
time_data = deque(maxlen=HISTORY_LENGTH)

# Dictionary holding separate deques for all 4 motors
history = {
    1: {'target': deque(maxlen=HISTORY_LENGTH), 'filt': deque(maxlen=HISTORY_LENGTH), 'raw': deque(maxlen=HISTORY_LENGTH)},
    2: {'target': deque(maxlen=HISTORY_LENGTH), 'filt': deque(maxlen=HISTORY_LENGTH), 'raw': deque(maxlen=HISTORY_LENGTH)},
    3: {'target': deque(maxlen=HISTORY_LENGTH), 'filt': deque(maxlen=HISTORY_LENGTH), 'raw': deque(maxlen=HISTORY_LENGTH)},
    4: {'target': deque(maxlen=HISTORY_LENGTH), 'filt': deque(maxlen=HISTORY_LENGTH), 'raw': deque(maxlen=HISTORY_LENGTH)}
}

# --- Globals for State and Thread Safety ---
system_running = True  
is_paused = False
ani = None 
current_motor_view = 1
update_motor_flag = 0  

# Shared dictionaries for UI parameters
trajectory_params = {"step_ms": 3000.0}

pid_params = {
    1: {"kp": 0.0400, "ki": 0.3000, "kd": 0.0020},
    2: {"kp": 0.0400, "ki": 0.3000, "kd": 0.0020},
    3: {"kp": 0.0400, "ki": 0.3000, "kd": 0.0020},
    4: {"kp": 0.0400, "ki": 0.3000, "kd": 0.0020}
}


class PIDController:
    def __init__(self, kp, ki, kd, ppr):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.ppr = ppr  # Store this specific motor's PPR
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        
        self.max_delta = 30.0
        self.pwm_max = 255.0
        self.pwm_min = 0.0
        
        self.rpm_filtered = 0.0
        self.last_raw_rpm = 0.0
        self.rpm_alpha = 0.3 

        self.pulse_history = deque(maxlen=10) 

    def update_gains(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def reset_state(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.pulse_history.clear()

    def compute(self, target_rpm, current_pulses):
        self.pulse_history.append(current_pulses)
        
        # Calculate RPM using the specific motor's PPR
        total_pulses_window = sum(self.pulse_history)
        window_time = DT * len(self.pulse_history) 
        rpm_raw = 0.0
        if window_time > 0:
            rpm_raw = (total_pulses_window / self.ppr) * (1.0 / window_time) * 60.0
            
        self.last_raw_rpm = rpm_raw
        
        # Exponential smoothing
        self.rpm_filtered = (self.rpm_filtered * (1.0 - self.rpm_alpha)) + (rpm_raw * self.rpm_alpha)
        
        error = target_rpm - self.rpm_filtered
        self.integral += error * DT
        derivative = (error - self.last_error) / DT
        
        raw_pid = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        
        delta = raw_pid - self.last_output
        if delta > self.max_delta:
            raw_pid = self.last_output + self.max_delta
        elif delta < -self.max_delta:
            raw_pid = self.last_output - self.max_delta
            
        pid_clamped = max(self.pwm_min, min(self.pwm_max, raw_pid))
        
        self.last_error = error
        self.last_output = pid_clamped
        
        direction = 1 if target_rpm >= 0 else 0
        return int(pid_clamped), direction


def control_loop():
    global system_running, update_motor_flag
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) 
        ser.reset_input_buffer() 
        print("Connected to ESP32. Starting control loop...")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        system_running = False
        return

    # Pass the specific PPR to each motor's controller upon initialization
    motors = {
        1: PIDController(pid_params[1]["kp"], pid_params[1]["ki"], pid_params[1]["kd"], MOTOR_PPR[1]),
        2: PIDController(pid_params[2]["kp"], pid_params[2]["ki"], pid_params[2]["kd"], MOTOR_PPR[2]),
        3: PIDController(pid_params[3]["kp"], pid_params[3]["ki"], pid_params[3]["kd"], MOTOR_PPR[3]),
        4: PIDController(pid_params[4]["kp"], pid_params[4]["ki"], pid_params[4]["kd"], MOTOR_PPR[4])
    }
    
    start_time = time.time()
    c1 = c2 = c3 = c4 = 0
    esp_is_reversed = 0 

    try:
        while system_running:
            # --- GUI UPDATE INTERRUPT ---
            if update_motor_flag > 0:
                m_id = update_motor_flag
                print(f"Updating Motor {m_id} PID Values... Halting motors temporarily.")
                ser.write("0,1,0,1,0,1,0,1\n".encode('utf-8')) 
                
                motors[m_id].update_gains(pid_params[m_id]["kp"], pid_params[m_id]["ki"], pid_params[m_id]["kd"])
                motors[m_id].reset_state()
                
                update_motor_flag = 0
                time.sleep(0.5) 
                ser.reset_input_buffer()
                continue
            
            # --- SERIAL PARSING ---
            if ser.in_waiting == 0:
                continue

            line = ser.readline().decode('utf-8', errors='ignore').strip()
            try:
                data = [int(x) for x in line.split(',') if x.lstrip('-').isdigit()]
                if len(data) == 5:
                    c1, c2, c3, c4, esp_is_reversed = data
                else:
                    continue
            except ValueError:
                continue 
            
            # --- TRAJECTORY PLANNING ---
            elapsed_ms = (time.time() - start_time) * 1000 
            
            current_step_ms = max(100.0, trajectory_params["step_ms"])
            phase = int((elapsed_ms / current_step_ms) % 2)
            
            t1 = 2000 if phase == 0 else 700
            t2, t3, t4 = t1, t1, 150 

            # --- COMPUTE PID ---
            pwm1, dir1 = motors[1].compute(t1, c1)
            pwm2, dir2 = motors[2].compute(t2, c2)
            pwm3, dir3 = motors[3].compute(t3, c3)
            pwm4, dir4 = motors[4].compute(t4, c4)

            if esp_is_reversed == 1:
                dir1, dir2, dir3, dir4 = 1 - dir1, 1 - dir2, 1 - dir3, 1 - dir4

            # --- WRITE COMMAND ---
            cmd = f"{pwm1},{dir1},{pwm2},{dir2},{pwm3},{dir3},{pwm4},{dir4}\n"
            ser.write(cmd.encode('utf-8'))
            
            # --- LOG DATA FOR ALL MOTORS ---
            time_data.append(elapsed_ms)
            
            history[1]['target'].append(t1); history[1]['filt'].append(motors[1].rpm_filtered); history[1]['raw'].append(motors[1].last_raw_rpm)
            history[2]['target'].append(t2); history[2]['filt'].append(motors[2].rpm_filtered); history[2]['raw'].append(motors[2].last_raw_rpm)
            history[3]['target'].append(t3); history[3]['filt'].append(motors[3].rpm_filtered); history[3]['raw'].append(motors[3].last_raw_rpm)
            history[4]['target'].append(t4); history[4]['filt'].append(motors[4].rpm_filtered); history[4]['raw'].append(motors[4].last_raw_rpm)

    finally:
        print("\nStopping motors...")
        ser.write("0,1,0,1,0,1,0,1\n".encode('utf-8'))
        ser.close()


def on_key_press(event):
    global is_paused, ani
    if event.inaxes in [ax_kp_t, ax_ki_t, ax_kd_t, ax_time_t]: return
    
    if event.key == ' ':
        is_paused = not is_paused
        if is_paused:
            ani.pause()
        else:
            ani.resume()
        event.canvas.draw()

def update_plot(frame, line_target, line_filtered, line_raw, ax):
    if not time_data: return line_target, line_filtered, line_raw
    
    t = list(time_data)
    targ = list(history[current_motor_view]['target'])
    filt = list(history[current_motor_view]['filt'])
    raw = list(history[current_motor_view]['raw'])
    
    line_target.set_data(t, targ)
    line_filtered.set_data(t, filt)
    line_raw.set_data(t, raw)
    
    if t[-1] > 0:
        ax.set_xlim(max(0, t[-1] - 10000), t[-1] + 500)
    ax.set_ylim(0, 4000) 
    return line_target, line_filtered, line_raw

def on_close(event):
    global system_running
    system_running = False

# --- GUI Setup Globals ---
ax_kp_t, ax_ki_t, ax_kd_t, ax_time_t = None, None, None, None

def main():
    global system_running, ani, ax_kp_t, ax_ki_t, ax_kd_t, ax_time_t, current_motor_view
    
    control_thread = threading.Thread(target=control_loop)
    control_thread.start()

    fig, ax = plt.subplots(figsize=(14, 6))
    plt.subplots_adjust(left=0.06, bottom=0.15, right=0.62)
    
    fig.canvas.mpl_connect('close_event', on_close) 
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    ax.set_title(f"Motor {current_motor_view} RPM -- LIVE (Press SPACE to pause)")
    ax.grid(True)
    
    line_raw, = ax.plot([], [], color='lightgray', alpha=0.7, label='Raw')
    line_target, = ax.plot([], [], 'r--', label='Target')
    line_filtered, = ax.plot([], [], 'b-', label='Filtered')
    ax.legend(loc="upper right")

    # --- Setup GUI Components ---
    ax_radio = plt.axes([0.70, 0.78, 0.22, 0.15], facecolor='lightgray')
    radio = RadioButtons(ax_radio, ('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'))

    ax_time_s = plt.axes([0.72, 0.65, 0.12, 0.03])
    ax_kp_s = plt.axes([0.72, 0.55, 0.12, 0.03])
    ax_ki_s = plt.axes([0.72, 0.45, 0.12, 0.03])
    ax_kd_s = plt.axes([0.72, 0.35, 0.12, 0.03])
    
    ax_time_t = plt.axes([0.86, 0.65, 0.08, 0.03])
    ax_kp_t = plt.axes([0.86, 0.55, 0.08, 0.03])
    ax_ki_t = plt.axes([0.86, 0.45, 0.08, 0.03])
    ax_kd_t = plt.axes([0.86, 0.35, 0.08, 0.03])
    
    ax_btn = plt.axes([0.70, 0.20, 0.24, 0.08])

    s_time = Slider(ax_time_s, 'Step(ms)', 500, 10000, valinit=trajectory_params["step_ms"], valstep=100)
    s_kp = Slider(ax_kp_s, 'Kp', 0.0, 1.0, valinit=pid_params[1]["kp"])
    s_ki = Slider(ax_ki_s, 'Ki', 0.0, 2.0, valinit=pid_params[1]["ki"])
    s_kd = Slider(ax_kd_s, 'Kd', 0.0, 0.1, valinit=pid_params[1]["kd"])
    
    t_time = TextBox(ax_time_t, '', initial=f'{trajectory_params["step_ms"]:.0f}')
    t_kp = TextBox(ax_kp_t, '', initial=f'{pid_params[1]["kp"]:.4f}')
    t_ki = TextBox(ax_ki_t, '', initial=f'{pid_params[1]["ki"]:.4f}')
    t_kd = TextBox(ax_kd_t, '', initial=f'{pid_params[1]["kd"]:.4f}')
    
    btn_update = Button(ax_btn, 'Update Settings (Stops Motor)', color='lightcoral', hovercolor='red')

    def select_motor(label):
        global current_motor_view
        current_motor_view = int(label.split(' ')[1])
        
        s_kp.set_val(pid_params[current_motor_view]["kp"])
        s_ki.set_val(pid_params[current_motor_view]["ki"])
        s_kd.set_val(pid_params[current_motor_view]["kd"])
        
        ax.set_title(f"Motor {current_motor_view} RPM -- LIVE (Press SPACE to pause)")
        fig.canvas.draw_idle()

    radio.on_clicked(select_motor)

    def sync_time_text(text):
        try: s_time.set_val(float(text))
        except ValueError: t_time.set_val(f"{s_time.val:.0f}")

    def sync_kp_text(text):
        try: s_kp.set_val(float(text))
        except ValueError: t_kp.set_val(f"{s_kp.val:.4f}")
            
    def sync_ki_text(text):
        try: s_ki.set_val(float(text))
        except ValueError: t_ki.set_val(f"{s_ki.val:.4f}")

    def sync_kd_text(text):
        try: s_kd.set_val(float(text))
        except ValueError: t_kd.set_val(f"{s_kd.val:.4f}")

    def sync_time_slider(val): t_time.set_val(f"{val:.0f}")
    def sync_kp_slider(val): t_kp.set_val(f"{val:.4f}")
    def sync_ki_slider(val): t_ki.set_val(f"{val:.4f}")
    def sync_kd_slider(val): t_kd.set_val(f"{val:.4f}")

    t_time.on_submit(sync_time_text)
    t_kp.on_submit(sync_kp_text)
    t_ki.on_submit(sync_ki_text)
    t_kd.on_submit(sync_kd_text)
    
    s_time.on_changed(sync_time_slider)
    s_kp.on_changed(sync_kp_slider)
    s_ki.on_changed(sync_ki_slider)
    s_kd.on_changed(sync_kd_slider)

    def on_update_clicked(event):
        global update_motor_flag
        trajectory_params["step_ms"] = s_time.val

        pid_params[current_motor_view]["kp"] = s_kp.val
        pid_params[current_motor_view]["ki"] = s_ki.val
        pid_params[current_motor_view]["kd"] = s_kd.val
        
        update_motor_flag = current_motor_view
        
    btn_update.on_clicked(on_update_clicked)

    ani = animation.FuncAnimation(fig, update_plot, fargs=(line_target, line_filtered, line_raw, ax),
                                  interval=50, blit=False, cache_frame_data=False)
    plt.show()
    
    system_running = False
    control_thread.join()

if __name__ == '__main__':
    main()