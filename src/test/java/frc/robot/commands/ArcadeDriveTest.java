package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class ArcadeDriveTest {
  @Test
  void fullControllerInputUsesConfiguredDrivetrainLimits() {
    ChassisSpeeds speeds = ArcadeDrive.toChassisSpeeds(1.0, 1.0, 1.0);

    assertEquals(
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        speeds.vxMetersPerSecond,
        1e-9);
    assertEquals(
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        speeds.vyMetersPerSecond,
        1e-9);
    assertEquals(
        Constants.SwerveConstants.kMaxAngularSpeedRadPerSec,
        speeds.omegaRadiansPerSecond,
        1e-9);
  }
}
