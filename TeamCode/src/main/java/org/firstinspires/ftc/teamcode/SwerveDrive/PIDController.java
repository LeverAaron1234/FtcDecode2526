package org.firstinspires.ftc.teamcode.SwerveDrive;

/// Simple AI generated PID controller, so I can stop building new ones :)


public class PIDController {
  // Gain coefficients
  private final double kp;
  private final double ki;
  private final double kd;

  // Output limits
  private final double minOutput;
  private final double maxOutput;

  // Persistent state variables
  private double target = 0.0;
  private double integralSum = 0.0;
  private double lastError = 0.0;
  private boolean firstRun = true;

  /**
   * Constructs a PID controller with specific gains and output clipping boundaries.
   */
  public PIDController(double kp, double ki, double kd, double minOutput, double maxOutput) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.minOutput = minOutput;
    this.maxOutput = maxOutput;
  }

  public void setTarget(double target) {
    this.target = target;
  }

  /**
   * Calculates the control output based on the current process variable and elapsed time.
   * @param current The measured value from the sensor.
   * @param dt The time elapsed since the last calculation in seconds (e.g., 0.02 for 50Hz).
   * @return The constrained control output.
   */
  public double calculate(double current, double dt) {
    if (dt <= 0.0) return minOutput; // Protect against division by zero

    // 1. Calculate the current tracking error
    double error = target - current;

    // 2. Proportional term
    double pTerm = kp * error;

    // 3. Integral term with anti-windup (only accumulates if not saturated)
    integralSum += error * dt;
    double iTerm = ki * integralSum;

    // 4. Derivative term (accounts for sudden changes over time)
    double derivative = firstRun ? 0.0 : (error - lastError) / dt;
    double dTerm = kd * derivative;

    firstRun = false;
    lastError = error;

    // 5. Total output summation
    double output = pTerm + iTerm + dTerm;

    // 6. Clamp output to hardware limits and apply anti-windup tracking
    if (output > maxOutput) {
      integralSum -= error * dt; // Undo integration to prevent windup
      return maxOutput;
    } else if (output < minOutput) {
      integralSum -= error * dt; // Undo integration to prevent windup
      return minOutput;
    }

    return output;
  }

  /**
   * Resets the internal error history (useful when restarting a mechanism profile).
   */
  public void reset() {
    this.integralSum = 0.0;
    this.lastError = 0.0;
    this.firstRun = true;
  }
}

