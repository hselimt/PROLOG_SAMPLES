# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

# ===========================
# DC MOTOR PARAMETERS
# ===========================
R = 1.0      # Resistance (Ohms)
L = 0.5      # Inductance (H)
Kt = 0.01    # Torque constant
Kb = 0.01    # Back-EMF constant
J = 0.01     # Inertia (kg*m^2)
B = 0.001    # Friction coefficient
Vmax = 24.0  # Maximum voltage

# ===========================
# FUZZY MEMBERSHIP FUNCTION
# ===========================
def triangular(x, a, b, c):
    """Triangular membership function with peak at b"""
    x = np.asarray(x)
    mu = np.zeros_like(x, dtype=float)
    # Rising edge: from a to b
    left = (a < x) & (x <= b)
    mu[left] = (x[left] - a) / (b - a + 1e-12)
    # Falling edge: from b to c
    right = (b < x) & (x < c)
    mu[right] = (c - x[right]) / (c - b + 1e-12)
    mu[x == b] = 1.0  # Peak value
    return mu

# ===========================
# FUZZY SETS DEFINITION
# ===========================
# Error fuzzy sets (NB=Negative Big, NS=Negative Small, Z=Zero, PS=Positive Small, PB=Positive Big)
max_e = 100.0
error_sets = {
    'NB': (-120, -100, -60),
    'NS': (-80, -40, 0),
    'Z':  (-10, 0, 10),
    'PS': (0, 40, 80),
    'PB': (60, 100, 120)
}

# Error derivative fuzzy sets (N=Negative, Z=Zero, P=Positive)
max_de = 400.0
deriv_sets = {
    'N': (-480, -320, -80),
    'Z': (-120, 0, 120),
    'P': (80, 320, 480)
}

# Output voltage fuzzy sets
voltage_sets = {
    'NB': (-28.8, -19.2, -9.6),
    'NS': (-12, -6, 0),
    'Z':  (-2.4, 0, 2.4),
    'PS': (0, 6, 12),
    'PB': (9.6, 19.2, 28.8)
}

# Fuzzy rule table: [error][derivative] -> output
rules = [
    ['NB', 'NB', 'NS'],  # If error is NB and derivative is N/Z/P
    ['NB', 'NS', 'Z'],   # If error is NS and derivative is N/Z/P
    ['NS', 'Z', 'PS'],   # If error is Z and derivative is N/Z/P
    ['Z', 'PS', 'PB'],   # If error is PS and derivative is N/Z/P
    ['PS', 'PB', 'PB']   # If error is PB and derivative is N/Z/P
]

# ===========================
# FUZZY CONTROLLER
# ===========================
def fuzzy_controller(error, error_deriv):
    """Mamdani fuzzy controller with centroid defuzzification"""
    # Fuzzification: calculate membership degrees
    mu_error = {label: triangular([error], *params)[0] for label, params in error_sets.items()}
    mu_deriv = {label: triangular([error_deriv], *params)[0] for label, params in deriv_sets.items()}
    
    # Inference and aggregation
    u_range = np.linspace(-Vmax, Vmax, 1001)  # Output universe
    output = np.zeros_like(u_range)
    
    error_labels = ['NB', 'NS', 'Z', 'PS', 'PB']
    deriv_labels = ['N', 'Z', 'P']
    
    for i, e_label in enumerate(error_labels):
        for j, d_label in enumerate(deriv_labels):
            # Calculate rule firing strength (minimum t-norm)
            strength = min(mu_error[e_label], mu_deriv[d_label])
            if strength > 0:
                # Get consequent fuzzy set
                out_label = rules[i][j]
                mu_consequent = triangular(u_range, *voltage_sets[out_label])
                # Aggregate using maximum (union)
                output = np.maximum(output, np.minimum(mu_consequent, strength))
    
    # Defuzzification: centroid method
    numerator = np.sum(u_range * output)
    denominator = np.sum(output)
    return 0.0 if denominator == 0 else numerator / denominator

# ===========================
# DC MOTOR DYNAMICS
# ===========================
def motor_dynamics(state, voltage):
    """Calculate derivatives for motor state [current, speed]"""
    current, speed = state
    # Current derivative: di/dt = (-R*i - Kb*w + V) / L
    di_dt = (-R * current - Kb * speed + voltage) / L
    # Speed derivative: dw/dt = (Kt*i - B*w) / J
    dw_dt = (Kt * current - B * speed) / J
    return np.array([di_dt, dw_dt])

def rk4_step(state, voltage, dt):
    """4th order Runge-Kutta integration step"""
    k1 = motor_dynamics(state, voltage)
    k2 = motor_dynamics(state + 0.5 * dt * k1, voltage)
    k3 = motor_dynamics(state + 0.5 * dt * k2, voltage)
    k4 = motor_dynamics(state + dt * k3, voltage)
    return state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

# ===========================
# SIMULATION
# ===========================
def simulate(reference_func, duration=10.0, dt=0.001, Ki=2.0, Kf=1.0):
    """
    Simulate fuzzy + integral control of DC motor
    reference_func: function returning desired speed at time t
    Ki: integral gain
    Kf: fuzzy controller gain
    """
    state = np.array([0.0, 0.0])  # Initial state: [current, speed]
    time = np.arange(0, duration + dt, dt)
    N = len(time)
    
    # History arrays
    speed = np.zeros(N)
    voltage = np.zeros(N)
    error = np.zeros(N)
    reference = np.zeros(N)
    
    prev_error = 0.0
    integral = 0.0
    
    for k in range(N):
        t = time[k]
        ref = reference_func(t)
        err = ref - state[1]  # Speed error
        err_deriv = (err - prev_error) / dt  # Error derivative
        
        # Special stabilization after 6.5 seconds
        if t >= 6.5:
            # Calculate steady-state voltage needed to maintain speed
            # Formula: V_ss = ref * ((R*B)/Kt + Kb)
            steady_voltage = ref * ((R * B / Kt) + Kb)
            v_total = steady_voltage
            # Keep integral aligned
            integral = (steady_voltage - Kf * fuzzy_controller(err, err_deriv)) / (Ki + 1e-6)
        else:
            # Normal fuzzy + integral control
            u_fuzzy = fuzzy_controller(err, err_deriv)
            integral += err * dt
            v_total = Kf * u_fuzzy + Ki * integral
        
        # Voltage saturation
        v_clipped = np.clip(v_total, -Vmax, Vmax)
        
        # Anti-windup: prevent integral accumulation when saturated
        if t < 6.5 and v_total != v_clipped:
            integral -= err * dt
        
        # Update motor state
        state = rk4_step(state, v_clipped, dt)
        
        # Store history
        speed[k] = state[1]
        voltage[k] = v_clipped
        error[k] = err
        reference[k] = ref
        prev_error = err
    
    return time, speed, voltage, error, reference

# ===========================
# MAIN EXECUTION
# ===========================
if __name__ == "__main__":
    # Define reference speed: ramp up to 100 rad/s in 0.2 seconds
    target_speed = 100.0
    def reference(t):
        return target_speed * (t / 0.2) if t < 0.2 else target_speed
    
    # Run simulation
    t, w, u, e, ref = simulate(reference, duration=10, dt=0.001, Ki=2.0, Kf=1.0)
    
    # Plot results
    plt.figure(figsize=(10, 8))
    
    # Speed tracking
    plt.subplot(3, 1, 1)
    plt.plot(t, ref, '--', color='red', label='Reference', linewidth=2)
    plt.plot(t, w, color='blue', label='Motor Speed', linewidth=2)
    plt.axvline(x=6.5, color='green', linestyle=':', label='Stabilization Point')
    plt.ylabel('Speed (rad/s)')
    plt.title('DC Motor Fuzzy Control with Stabilization')
    plt.legend()
    plt.grid(True)
    
    # Control voltage
    plt.subplot(3, 1, 2)
    plt.plot(t, u, color='orange', linewidth=1.5)
    plt.ylabel('Voltage (V)')
    plt.grid(True)
    
    # Tracking error
    plt.subplot(3, 1, 3)
    plt.plot(t, e, color='purple', linewidth=1.5)
    plt.ylabel('Error (rad/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
