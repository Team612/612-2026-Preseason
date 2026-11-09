package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

/** Per-module voltage history, with bounded elapsed time and watchdog-gap recovery. */
final class DriveVoltageLimiter {
  private final double rate;
  private final double maxStepSeconds;
  private final double expirationSeconds;
  private final DoubleSupplier clock;
  private double voltage;
  private double timestamp;

  DriveVoltageLimiter(double rate, double maxStepSeconds, double expirationSeconds,
      DoubleSupplier clock) {
    if (!Double.isFinite(rate) || rate < 0.0
        || !Double.isFinite(maxStepSeconds) || maxStepSeconds <= 0.0
        || !Double.isFinite(expirationSeconds) || expirationSeconds <= maxStepSeconds) {
      throw new IllegalArgumentException("Invalid drive voltage slew rate or timing limits");
    }
    this.rate = rate;
    this.maxStepSeconds = maxStepSeconds;
    this.expirationSeconds = expirationSeconds;
    this.clock = java.util.Objects.requireNonNull(clock);
    reset();
  }

  double calculate(double targetVolts) {
    double now = clock.getAsDouble();
    double elapsed = now - timestamp;
    timestamp = now;
    if (!Double.isFinite(targetVolts) || !Double.isFinite(elapsed)
        || elapsed < 0.0 || elapsed >= expirationSeconds) {
      // A watchdog may have neutralized the motor. Never resume from stale output history.
      voltage = 0.0;
      return voltage;
    }
    double step = rate * Math.min(elapsed, maxStepSeconds);
    voltage = rate == 0.0 ? targetVolts
        : voltage + MathUtil.clamp(targetVolts - voltage, -step, step);
    return voltage;
  }

  /** Explicit stops bypass the ramp and clear both output and elapsed-time history. */
  void reset() {
    voltage = 0.0;
    timestamp = clock.getAsDouble();
  }
}
