import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# 1) DC Motor parameters
# ---------------------------
R = 1.0
L = 0.5
Kt = 0.01
Kb = 0.01
J = 0.01
B = 0.001
Vmax = 24.0

# ---------------------------
# 2) Membership functions (same as previous setup)
# ---------------------------
def triangular(x, a, b, c):
    x = np.asarray(x)
    mu = np.zeros_like(x, dtype=float)
    left = (a < x) & (x <= b)
    mu[left] = (x[left] - a) / (b - a + 1e-12)
    right = (b < x) & (x < c)
    mu[right] = (c - x[right]) / (c - b + 1e-12)
    mu[x == b] = 1.0
    return mu

# --- Error (e) membership functions (5 sets) ---
max_err = 100.0
err_NB = (-max_err*1.2, -max_err, -max_err*0.6)
err_NS = (-max_err*0.8, -max_err*0.4, 0)
err_Z  = (-max_err*0.1, 0, max_err*0.1)
err_PS = (0, max_err*0.4, max_err*0.8)
err_PB = (max_err*0.6, max_err, max_err*1.2)

# --- Error rate (de/dt) membership functions (3 sets) ---
max_derr = 400.0
derr_N = (-max_derr*1.2, -max_derr*0.8, -max_derr*0.2)
derr_Z = (-max_derr*0.3, 0, max_derr*0.3)
derr_P = (max_derr*0.2, max_derr*0.8, max_derr*1.2)

# --- Output (u) membership sets (Zero-Centered System) ---
ctrl_NB = (-Vmax*1.2, -Vmax*0.8, -Vmax*0.4)
ctrl_NS = (-Vmax*0.5, -Vmax*0.25, 0)
ctrl_Z  = (-Vmax*0.1, 0, Vmax*0.1)
ctrl_PS = (0, Vmax*0.25, Vmax*0.5)
ctrl_PB = (Vmax*0.4, Vmax*0.8, Vmax*1.2)

# --- 5x3 rule base (e x de) ---
fuzzy_rules = [
    ['NB', 'NB', 'NS'],
    ['NB', 'NS', 'Z'],
    ['NS', 'Z', 'PS'],
    ['Z', 'PS', 'PB'],
    ['PS', 'PB', 'PB']
]

# Output membership function dictionary
ctrl_mfs = {'NB': ctrl_NB, 'NS': ctrl_NS, 'Z': ctrl_Z, 'PS': ctrl_PS, 'PB': ctrl_PB}

# --- Fuzzification ---
def fuzzify_inputs(err, err_rate):
    mu_err = {
        'NB': triangular([err], *err_NB)[0],
        'NS': triangular([err], *err_NS)[0],
        'Z':  triangular([err], *err_Z)[0],
        'PS': triangular([err], *err_PS)[0],
        'PB': triangular([err], *err_PB)[0],
    }
    mu_derr = {'N': triangular([err_rate], *derr_N)[0], 'Z': triangular([err_rate], *derr_Z)[0], 'P': triangular([err_rate], *derr_P)[0]}
    return mu_err, mu_derr

# --- Mamdani Defuzzification ---
def mamdani_inference(err, err_rate, ctrl_range=np.linspace(-Vmax, Vmax, 1001)):
    mu_err, mu_derr = fuzzify_inputs(err, err_rate)
    result_aggregate = np.zeros_like(ctrl_range)

    err_labels = ['NB', 'NS', 'Z', 'PS', 'PB']
    derr_labels = ['N', 'Z', 'P']

    for idx_err, err_lab in enumerate(err_labels):
        for idx_derr, derr_lab in enumerate(derr_labels):
            activation = min(mu_err[err_lab], mu_derr[derr_lab])
            if activation <= 0: continue

            ctrl_label = fuzzy_rules[idx_err][idx_derr]
            a, b, c = ctrl_mfs[ctrl_label]

            mu_ctrl = triangular(ctrl_range, a, b, c)
            result_aggregate = np.maximum(result_aggregate, np.minimum(mu_ctrl, activation))

    numerator, denominator = np.sum(ctrl_range * result_aggregate), np.sum(result_aggregate)
    return 0.0 if denominator == 0 else numerator/denominator

# ---------------------------
# 3) DC Motor dynamics
# ---------------------------
def compute_derivatives(state, voltage, load_torque=0.0):
    current, speed = state
    di = (-R*current - Kb*speed + voltage)/L
    dw = (-B*speed + Kt*current - load_torque)/J
    return np.array([di, dw])

def rk4_integration(state, voltage, timestep, load_torque=0.0):
    k1 = compute_derivatives(state, voltage, load_torque)
    k2 = compute_derivatives(state + 0.5*timestep*k1, voltage, load_torque)
    k3 = compute_derivatives(state + 0.5*timestep*k2, voltage, load_torque)
    k4 = compute_derivatives(state + timestep*k3, voltage, load_torque)
    return state + (timestep/6)*(k1 + 2*k2 + 2*k3 + k4)

# ---------------------------
# 4) Closed-loop simulation
# ---------------------------
def run_simulation(setpoint_func, T=5.0, dt=0.001, initial_state=None):
    if initial_state is None: state = np.array([0.0, 0.0])
    else: state = np.array(initial_state, dtype=float)
    time_array = np.arange(0, T+dt, dt)
    N = len(time_array)
    current_log, speed_log, voltage_log, error_log, setpoint_log = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N)

    prev_error = setpoint_func(0.0) - state[1]

    error_integral = 0.0
    Ki = 8.0
    K_fuzzy = 1.0
    for idx in range(N):
        setpoint = setpoint_func(time_array[idx])
        error = setpoint - state[1]
        error_rate = (error - prev_error) / dt

        fuzzy_output = mamdani_inference(error, error_rate)
        error_integral += error * dt
        total_voltage = (K_fuzzy * fuzzy_output) + (Ki * error_integral)

        voltage_limited = np.clip(total_voltage, -Vmax, Vmax)

        if total_voltage != voltage_limited:
            error_integral -= error * dt

        state = rk4_integration(state, voltage_limited, dt)
        current_log[idx], speed_log[idx], voltage_log[idx], error_log[idx], setpoint_log[idx] = state[0], state[1], voltage_limited, error, setpoint
        prev_error = error

    return time_array, speed_log, voltage_log, error_log, setpoint_log

# ---------------------------
# 5) Example simulation (*** Updated to T=6.5 ***)
# ---------------------------
if __name__ == "__main__":
    target_speed = 100.0  # rad/s
    def setpoint(t):
        return target_speed * (t/0.2) if t < 0.2 else target_speed

    # *** CHANGE HERE ***
    time, speed, voltage, error, reference = run_simulation(setpoint, T=6.5, dt=0.001) # Changed from 10.0 to 6.5

    plt.figure(figsize=(10,6))
    plt.subplot(3,1,1)
    plt.plot(time, reference, '--', label='Reference')
    plt.plot(time, speed, label='Angular Velocity')
    plt.ylabel('Speed (rad/s)'); plt.legend(); plt.grid(True)
    plt.title('Simulation Results (6.5 Seconds)')

    plt.subplot(3,1,2)
    plt.plot(time, voltage); plt.ylabel('Control Voltage (V)'); plt.grid(True)
    plt.ylim(-Vmax*0.1, Vmax*1.1)

    plt.subplot(3,1,3)
    plt.plot(time, error); plt.ylabel('Tracking Error'); plt.xlabel('Time (s)'); plt.grid(True)

    plt.tight_layout(); plt.show()


    # ============================================================
    # 6) ALL MEMBERSHIP FUNCTION PLOTS
    # ============================================================
    # a) Error (e) membership functions
    e_range = np.linspace(-120, 120, 800)
    plt.figure(figsize=(8,4))
    plt.plot(e_range, triangular(e_range, *err_NB), label='NB')
    plt.plot(e_range, triangular(e_range, *err_NS), label='NS')
    plt.plot(e_range, triangular(e_range, *err_Z),  label='Z')
    plt.plot(e_range, triangular(e_range, *err_PS), label='PS')
    plt.plot(e_range, triangular(e_range, *err_PB), label='PB')
    plt.title('Error (e) Membership Functions')
    plt.xlabel('Error'); plt.ylabel('Membership Degree')
    plt.legend(); plt.grid(True); plt.tight_layout()

    # b) Error rate (de/dt) membership functions
    de_range = np.linspace(-480, 480, 600)
    plt.figure(figsize=(8,4))
    plt.plot(de_range, triangular(de_range, *derr_N), label='N')
    plt.plot(de_range, triangular(de_range, *derr_Z), label='Z')
    plt.plot(de_range, triangular(de_range, *derr_P), label='P')
    plt.title('Error Derivative (de/dt) Membership Functions')
    plt.xlabel('Error Rate'); plt.ylabel('Membership Degree')
    plt.legend(); plt.grid(True); plt.tight_layout()

    # c) Control output (u) membership functions
    u_range = np.linspace(-Vmax*1.2, Vmax*1.2, 600)
    plt.figure(figsize=(8,4))
    plt.plot(u_range, triangular(u_range, *ctrl_NB), label='NB')
    plt.plot(u_range, triangular(u_range, *ctrl_NS), label='NS')
    plt.plot(u_range, triangular(u_range, *ctrl_Z),  label='Z (0V)')
    plt.plot(u_range, triangular(u_range, *ctrl_PS), label='PS')
    plt.plot(u_range, triangular(u_range, *ctrl_PB), label='PB')
    plt.title('Control Output (u) Membership Functions (Zero-Centered)')
    plt.xlabel('Control Voltage (u)')
    plt.ylabel('Membership Degree')
    plt.legend(); plt.grid(True)
    plt.tight_layout()

    plt.show()
