package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class DriveInputLimiterTest {
  private static final double SPEED = Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
  private static final double OMEGA = Constants.SwerveConstants.kMaxAngularSpeedRadPerSec;

  @Test
  void diagonalSharesAccelerationBudget() {
    var limiter = new DriveInputLimiter();
    var speed = limiter.calculate(1, 1, 0, 0.02);
    assertEquals(0.16 * SPEED, Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond), 1e-9);
    assertEquals(speed.vxMetersPerSecond, speed.vyMetersPerSecond, 1e-9);
  }

  @Test
  void centeredSticksReleaseFasterWithoutOvershoot() {
    var limiter = new DriveInputLimiter();
    for (int i = 0; i < 10; i++) limiter.calculate(-1, 0, -1, 0.02);
    var speed = limiter.calculate(0.001, 0, 0.001, 0.02);
    assertEquals(-0.8 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(-0.8 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    for (int i = 0; i < 6; i++) speed = limiter.calculate(0, 0, 0, 0.02);
    assertStopped(speed);
  }

  @Test
  void longAndInvalidTimeStepsCannotJumpDemand() {
    var limiter = new DriveInputLimiter();
    var speed = limiter.calculate(1, 0, 1, 1.0);
    assertEquals(0.32 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0.32 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    for (double time : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
      var unchanged = limiter.calculate(1, 0, 1, time);
      assertEquals(speed.vxMetersPerSecond, unchanged.vxMetersPerSecond, 1e-9);
      assertEquals(speed.omegaRadiansPerSecond, unchanged.omegaRadiansPerSecond, 1e-9);
    }
  }

  @Test
  void reversalUsesNormalRateAndResetClearsDemand() {
    var limiter = new DriveInputLimiter();
    for (int i = 0; i < 10; i++) limiter.calculate(1, 0, 1, 0.02);
    var speed = limiter.calculate(-1, 0, -1, 0.02);
    assertEquals(0.84 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0.84 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    limiter.reset();
    assertStopped(limiter.calculate(0, 0, 0, 0.02));
  }

  @Test
  void invalidInputStopsAndDoesNotPoisonFutureOutput() {
    var limiter = new DriveInputLimiter();
    limiter.calculate(1, 1, 1, 0.02);
    assertStopped(limiter.calculate(Double.NaN, 0, 0, 0.02));
    var speed = limiter.calculate(1, 0, 0, 0.02);
    assertEquals(0.16 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertStopped(limiter.calculate(0, Double.POSITIVE_INFINITY, 0, 0.02));
    assertStopped(limiter.calculate(0, 0, Double.NaN, 0.02));
  }

  private static void assertStopped(ChassisSpeeds speed) {
    assertEquals(0, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0, speed.vyMetersPerSecond, 1e-9);
    assertEquals(0, speed.omegaRadiansPerSecond, 1e-9);
  }
}
