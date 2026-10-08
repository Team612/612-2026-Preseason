package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class ArcadeDriveTest {
  @Test
  void diagonalControllerInputIsNormalizedToConfiguredLinearSpeed() {
    ChassisSpeeds speeds = ArcadeDrive.toChassisSpeeds(1.0, 1.0, 1.0);

    assertEquals(
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond / Math.sqrt(2.0),
        speeds.vxMetersPerSecond,
        1e-9);
    assertEquals(
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond / Math.sqrt(2.0),
        speeds.vyMetersPerSecond,
        1e-9);
    assertEquals(
        Constants.SwerveConstants.kMaxAngularSpeedRadPerSec,
        speeds.omegaRadiansPerSecond,
        1e-9);
  }

  @Test
  void controllerInputOutsideNormalizedRangeIsClamped() {
    ChassisSpeeds speeds = ArcadeDrive.toChassisSpeeds(2.0, 0.0, -2.0);

    assertEquals(
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        speeds.vxMetersPerSecond,
        1e-9);
    assertEquals(
        -Constants.SwerveConstants.kMaxAngularSpeedRadPerSec,
        speeds.omegaRadiansPerSecond,
        1e-9);
  }

  @Test
  void translationDeadbandAndDiagonalNormalizationKeepInputsBounded() {
    Translation2d belowDeadband =
        ArcadeDrive.applyTranslationDeadband(0.03, -0.04, Constants.OperatorConstants.kDeadband);
    Translation2d diagonal =
        ArcadeDrive.applyTranslationDeadband(1.0, 1.0, Constants.OperatorConstants.kDeadband);

    assertEquals(0.0, belowDeadband.getX(), 1e-9);
    assertEquals(0.0, belowDeadband.getY(), 1e-9);
    assertEquals(1.0, Math.hypot(diagonal.getX(), diagonal.getY()), 1e-9);
  }

  @Test
  void xboxAxesMapForwardLeftAndCounterClockwiseToPositiveSpeeds() {
    ChassisSpeeds forward = ArcadeDrive.fromControllerAxes(0.0, -1.0, 0.0);
    ChassisSpeeds left = ArcadeDrive.fromControllerAxes(-1.0, 0.0, 0.0);
    ChassisSpeeds counterClockwise = ArcadeDrive.fromControllerAxes(0.0, 0.0, -1.0);

    assertEquals(Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        forward.vxMetersPerSecond, 1e-9);
    assertEquals(Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        left.vyMetersPerSecond, 1e-9);
    assertEquals(Constants.SwerveConstants.kMaxAngularSpeedRadPerSec,
        counterClockwise.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void supplierInputsMapForwardStrafeAndRotationToRobotRelativeSpeeds() {
    ChassisSpeeds speeds = ArcadeDrive.fromControllerInputs(1.0, -1.0, 0.5);

    assertEquals(
        Constants.SwerveConstants.kMaxSpeedMetersPerSecond / Math.sqrt(2.0),
        speeds.vxMetersPerSecond,
        1e-9);
    assertEquals(
        -Constants.SwerveConstants.kMaxSpeedMetersPerSecond / Math.sqrt(2.0),
        speeds.vyMetersPerSecond,
        1e-9);
    assertEquals(
        ((0.5 - Constants.OperatorConstants.kDeadband)
                / (1.0 - Constants.OperatorConstants.kDeadband))
            * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec,
        speeds.omegaRadiansPerSecond,
        1e-9);
  }

  @Test
  void justAboveDeadbandProducesSmallNonzeroChassisSpeed() {
    double justAboveDeadband = Constants.OperatorConstants.kDeadband + 0.01;
    ChassisSpeeds speeds = ArcadeDrive.fromControllerAxes(0.0, -justAboveDeadband, 0.0);

    assertEquals(
        (justAboveDeadband - Constants.OperatorConstants.kDeadband)
            / (1.0 - Constants.OperatorConstants.kDeadband)
            * Constants.SwerveConstants.kMaxSpeedMetersPerSecond,
        speeds.vxMetersPerSecond,
        1e-9);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-9);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void smallStickDriftIsDeadbanded() {
    ChassisSpeeds speeds = ArcadeDrive.fromControllerAxes(0.08, -0.07, -0.09);

    assertEquals(0.0, speeds.vxMetersPerSecond, 1e-9);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-9);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-9);
  }

}
