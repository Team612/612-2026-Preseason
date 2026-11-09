package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class DriveInputLimiterTest {
  private static final double SPEED = Constants.SwerveConstants.kMaxSpeedMetersPerSecond;
  private static final double OMEGA = Constants.SwerveConstants.kMaxAngularSpeedRadPerSec;

  @Test
  void diagonalSharesAccelerationBudget() {
    var limiter = new DriveInputLimiter();
    var speed = limiter.calculate(1, 1, 0, 0.02);
    assertEquals(0.06 * SPEED, Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond), 1e-9);
    assertEquals(speed.vxMetersPerSecond, speed.vyMetersPerSecond, 1e-9);
  }

  @Test
  void centeredSticksReleaseFasterWithoutOvershoot() {
    var limiter = new DriveInputLimiter();
    ChassisSpeeds before = new ChassisSpeeds();
    for (int i = 0; i < 120; i++) before = limiter.calculate(-1, 0, -1, 0.02);
    var speed = limiter.calculate(0.001, 0, 0.001, 0.02);
    assertEquals(before.vxMetersPerSecond + 0.10 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(before.omegaRadiansPerSecond + 0.10 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    for (int i = 0; i < 60; i++) speed = limiter.calculate(0, 0, 0, 0.02);
    assertStopped(speed);
  }

  @Test
  void longAndInvalidTimeStepsCannotJumpDemand() {
    var limiter = new DriveInputLimiter();
    var speed = limiter.calculate(1, 0, 1, 1.0);
    assertEquals(0.06 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0.06 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    for (double time : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
      var unchanged = limiter.calculate(1, 0, 1, time);
      assertEquals(speed.vxMetersPerSecond, unchanged.vxMetersPerSecond, 1e-9);
      assertEquals(speed.omegaRadiansPerSecond, unchanged.omegaRadiansPerSecond, 1e-9);
    }
  }

  @Test
  void reversalUsesNormalRateAndResetClearsDemand() {
    var limiter = new DriveInputLimiter();
    ChassisSpeeds before = new ChassisSpeeds();
    for (int i = 0; i < 120; i++) before = limiter.calculate(1, 0, 1, 0.02);
    var speed = limiter.calculate(-1, 0, -1, 0.02);
    assertEquals(before.vxMetersPerSecond - 0.03 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(before.omegaRadiansPerSecond - 0.03 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    limiter.reset();
    assertStopped(limiter.calculate(0, 0, 0, 0.02));
  }

  @Test
  void invalidInputStopsAndDoesNotPoisonFutureOutput() {
    var limiter = new DriveInputLimiter();
    limiter.calculate(1, 1, 1, 0.02);
    assertStopped(limiter.calculate(Double.NaN, 0, 0, 0.02));
    var speed = limiter.calculate(1, 0, 0, 0.02);
    assertEquals(0.06 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertStopped(limiter.calculate(0, Double.POSITIVE_INFINITY, 0, 0.02));
    assertStopped(limiter.calculate(0, 0, Double.NaN, 0.02));
  }

  @Test
  void simultaneousTranslationAndRotationShareAccelerationBudget() {
    var limiter = new DriveInputLimiter();
    var speed = limiter.calculate(1, 0, 1, 0.02);
    assertEquals(0.03 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0.03 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    for (int i = 0; i < 120; i++) speed = limiter.calculate(1, 0, 1, 0.02);
    assertTrue(speed.vxMetersPerSecond > 0.0);
    assertTrue(speed.omegaRadiansPerSecond > 0.0);
    assertEquals(speed.vxMetersPerSecond / SPEED, speed.omegaRadiansPerSecond / OMEGA, 1e-9);
    double fastest = 0;
    for (var state : Constants.swerveKinematics.toSwerveModuleStates(speed)) {
      fastest = Math.max(fastest, Math.abs(state.speedMetersPerSecond));
    }
    assertTrue(fastest <= SPEED + 1e-9);
    assertEquals(1.0, speed.vxMetersPerSecond / SPEED
        + speed.omegaRadiansPerSecond / OMEGA, 1e-9);
  }

  @Test
  void addingRotationAtFullSpeedMakesRoomWithoutStoppingTranslation() {
    var limiter = new DriveInputLimiter();
    for (int i = 0; i < 60; i++) limiter.calculate(1, 0, 0, 0.02);
    var speed = limiter.calculate(1, 0, 1, 0.02);
    assertEquals(0.90 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0.06 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void combinedReversalBoundsEveryWheelVelocityChange() {
    var limiter = new DriveInputLimiter();
    ChassisSpeeds previous = new ChassisSpeeds();
    for (int i = 0; i < 240; i++) {
      double direction = i < 120 ? 1.0 : -1.0;
      var next = limiter.calculate(direction, direction, direction, 0.02);
      var before = Constants.swerveKinematics.toSwerveModuleStates(previous);
      var after = Constants.swerveKinematics.toSwerveModuleStates(next);
      for (int wheel = 0; wheel < before.length; wheel++) {
        double dx = after[wheel].speedMetersPerSecond * after[wheel].angle.getCos()
            - before[wheel].speedMetersPerSecond * before[wheel].angle.getCos();
        double dy = after[wheel].speedMetersPerSecond * after[wheel].angle.getSin()
            - before[wheel].speedMetersPerSecond * before[wheel].angle.getSin();
        assertTrue(Math.hypot(dx, dy) <= 0.06 * SPEED + 1e-9);
      }
      previous = next;
    }
  }

  @Test
  void combinedReleaseStopsWithinPointTwoSeconds() {
    var limiter = new DriveInputLimiter();
    for (int i = 0; i < 60; i++) limiter.calculate(1, 1, 1, 0.02);
    ChassisSpeeds speed = new ChassisSpeeds();
    for (int i = 0; i < 10; i++) speed = limiter.calculate(0, 0, 0, 0.02);
    assertStopped(speed);
  }

  @Test
  void partialSlowdownUsesBrakingRateWhileRotationAccelerates() {
    var limiter = new DriveInputLimiter();
    for (int i = 0; i < 60; i++) limiter.calculate(1, 0, 0, 0.02);
    var speed = limiter.calculate(0.5, 0, 1, 0.02);
    assertEquals(0.90 * SPEED, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0.06 * OMEGA, speed.omegaRadiansPerSecond, 1e-9);
    for (int i = 0; i < 10; i++) speed = limiter.calculate(0.5, 0, 1, 0.02);
    assertTrue(speed.vxMetersPerSecond > 0.0);
    for (var state : Constants.swerveKinematics.toSwerveModuleStates(speed)) {
      assertTrue(state.speedMetersPerSecond <= SPEED + 1e-9);
    }
  }

  @Test
  void combinedDemandDoesNotWindUpBeyondWheelCapability() {
    for (double forward : new double[] {-1, -0.4, 0, 0.4, 1}) {
      for (double strafe : new double[] {-1, 0, 1}) {
        for (double rotation : new double[] {-1, 0.5, 1}) {
          var limiter = new DriveInputLimiter();
          ChassisSpeeds speed = new ChassisSpeeds();
          for (int i = 0; i < 100; i++) speed = limiter.calculate(forward, strafe, rotation, 0.02);
          for (int heading = 0; heading < 360; heading += 15) {
            var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, Rotation2d.fromDegrees(heading));
            for (var wheel : Constants.swerveKinematics.toSwerveModuleStates(robotSpeeds)) {
              assertTrue(wheel.speedMetersPerSecond <= SPEED + 1e-9,
                  "Limiter history must stay achievable as the robot rotates");
            }
          }
          double before = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
          speed = limiter.calculate(forward, strafe, 0, 0.02);
          assertTrue(Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) - before
              <= Constants.OperatorConstants.kTranslationSlewRatePerSecond * SPEED * 0.02 + 1e-9);
        }
      }
    }
  }

  @Test
  void changingCombinedInputsRemainFeasibleDuringTheRamp() {
    var random = new java.util.Random(612);
    var limiter = new DriveInputLimiter();
    for (int request = 0; request < 200; request++) {
      double forward = random.nextDouble() * 2 - 1;
      double strafe = random.nextDouble() * 2 - 1;
      double rotation = random.nextDouble() * 2 - 1;
      for (int tick = 0; tick < 25; tick++) {
        var speed = limiter.calculate(forward, strafe, rotation, 0.02);
        double demand = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) / SPEED
            + Math.abs(speed.omegaRadiansPerSecond) / OMEGA;
        assertTrue(demand <= 1.0 + 1e-9, "Combined transient demand exceeded envelope: " + demand);
      }
    }
  }

  private static void assertStopped(ChassisSpeeds speed) {
    assertEquals(0, speed.vxMetersPerSecond, 1e-9);
    assertEquals(0, speed.vyMetersPerSecond, 1e-9);
    assertEquals(0, speed.omegaRadiansPerSecond, 1e-9);
  }
}
