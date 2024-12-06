#include <stdio.h>

// PID Controller structure
typedef struct {
    double kp; // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain
    double prev_error; // Previous error
    double integral; // Integral of error
} PIDController;

// Initialize the PID controller
void PID_Init(PIDController *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
}

// Compute the control signal
double PID_Compute(PIDController *pid, double setpoint, double measured_value, double dt) {
    double error = setpoint - measured_value;
    pid->integral += error * dt;
    double derivative = (error - pid->prev_error) / dt;
    double output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prev_error = error;
    return output;
}

int main(){
    PIDController pid;
    PID_Init(&pid, 1.0, 0.1, 0.01); // Initialize with some gains

    double setpoint = 100.0; // Desired value
    double measured_value = 90.0; // Initial measured value
    double dt = 0.1; // Time step

    for (int i = 0; i < 100; i++) {
        double control_signal = PID_Compute(&pid, setpoint, measured_value, dt);
        printf("Control Signal: %f\n", control_signal);
        measured_value += control_signal * dt; // Simulate system response
    }

    return 0;
}