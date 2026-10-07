package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class SwerveTest {
  @Test
  void currentCanMapPassesCanIdRangeValidation() {
    assertTrue(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      Constants.SwerveConstants.kFrontLeft,
      Constants.SwerveConstants.kFrontRight,
      Constants.SwerveConstants.kRearLeft,
      Constants.SwerveConstants.kRearRight
    }));
  }

  @Test
  void duplicateCanIdsAreNotValidatedHere() {
    assertTrue(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      new Constants.ModuleConfiguration(1, 2, 3, 0.0, false, false),
      new Constants.ModuleConfiguration(4, 5, 6, 0.0, false, false),
      new Constants.ModuleConfiguration(7, 8, 9, 0.0, false, false),
      new Constants.ModuleConfiguration(10, 11, 3, 0.0, false, false)
    }));
  }

  @Test
  void pigeonCanIdOverlapIsNotValidatedHere() {
    assertTrue(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      new Constants.ModuleConfiguration(0, 2, 3, 0.0, false, false),
      new Constants.ModuleConfiguration(4, 5, 6, 0.0, false, false),
      new Constants.ModuleConfiguration(7, 8, 9, 0.0, false, false),
      new Constants.ModuleConfiguration(10, 11, 12, 0.0, false, false)
    }));
  }

  @Test
  void straightForwardChassisCommandPointsAllWheelsForward() {
    SwerveModuleState[] states =
        Swerve.calculateModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));

    assertEquals(4, states.length);
    for (SwerveModuleState state : states) {
      assertEquals(1.0, state.speedMetersPerSecond, 1e-9);
      assertEquals(0.0, state.angle.getRadians(), 1e-9);
    }
  }

  @Test
  void straightStrafeChassisCommandPointsAllWheelsSideways() {
    SwerveModuleState[] states =
        Swerve.calculateModuleStates(new ChassisSpeeds(0.0, 1.0, 0.0));

    for (SwerveModuleState state : states) {
      assertEquals(1.0, state.speedMetersPerSecond, 1e-9);
      assertEquals(Math.PI / 2.0, state.angle.getRadians(), 1e-9);
    }
  }

  @Test
  void moduleOptimizationReversesDriveInsteadOfTurningMoreThanNinetyDegrees() {
    SwerveModuleState requested =
        new SwerveModuleState(1.0, Rotation2d.fromDegrees(170.0));

    SwerveModuleState optimized =
        SwerveModule.optimizeState(requested, Rotation2d.fromDegrees(0.0));

    assertEquals(-1.0, optimized.speedMetersPerSecond, 1e-9);
    assertEquals(-10.0, optimized.angle.getDegrees(), 1e-9);
  }

  @Test
  void moduleOptimizationKeepsDriveDirectionForShortSteeringTurn() {
    SwerveModuleState requested =
        new SwerveModuleState(1.0, Rotation2d.fromDegrees(45.0));

    SwerveModuleState optimized =
        SwerveModule.optimizeState(requested, Rotation2d.fromDegrees(0.0));

    assertEquals(1.0, optimized.speedMetersPerSecond, 1e-9);
    assertEquals(45.0, optimized.angle.getDegrees(), 1e-9);
  }

  @Test
  void incompleteCanMapsAreRejected() {
    assertFalse(Swerve.hasValidCanIds(null));
    assertFalse(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[3]));
    assertFalse(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      new Constants.ModuleConfiguration(1, 2, 3, 0.0, false, false),
      null,
      new Constants.ModuleConfiguration(7, 8, 9, 0.0, false, false),
      new Constants.ModuleConfiguration(10, 11, 12, 0.0, false, false)
    }));
  }

  @Test
  void validCanMapIsAccepted() {
    assertTrue(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      new Constants.ModuleConfiguration(1, 2, 3, 0.0, false, false),
      new Constants.ModuleConfiguration(4, 5, 6, 0.0, false, false),
      new Constants.ModuleConfiguration(7, 8, 9, 0.0, false, false),
      new Constants.ModuleConfiguration(10, 11, 12, 0.0, false, false)
    }));
  }

  @Test
  void canIdsOutsidePhoenixRangeAreRejected() {
    assertFalse(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      new Constants.ModuleConfiguration(1, 2, 3, 0.0, false, false),
      new Constants.ModuleConfiguration(4, 5, 6, 0.0, false, false),
      new Constants.ModuleConfiguration(7, 8, 9, 0.0, false, false),
      new Constants.ModuleConfiguration(10, 11, 63, 0.0, false, false)
    }));
  }

  @Test
  void zeroChassisCommandIsRecognizedAsStopped() {
    assertTrue(Swerve.isZeroCommand(new ChassisSpeeds()));
  }

  @Test
  void invalidModulePositionUsesLastTrustedPosition() {
    SwerveModulePosition[] previous = {
      new SwerveModulePosition(1.0, Rotation2d.fromDegrees(10.0)),
      new SwerveModulePosition(2.0, Rotation2d.fromDegrees(20.0))
    };
    SwerveModulePosition[] current = {
      new SwerveModulePosition(1.5, Rotation2d.fromDegrees(15.0)),
      null
    };

    SwerveModulePosition[] positions = Swerve.holdLastValidPositions(current, previous);

    assertEquals(1.5, positions[0].distanceMeters, 1e-9);
    assertEquals(15.0, positions[0].angle.getDegrees(), 1e-9);
    assertEquals(2.0, positions[1].distanceMeters, 1e-9);
    assertEquals(20.0, positions[1].angle.getDegrees(), 1e-9);
  }

  @Test
  void nonzeroChassisCommandIsNotTreatedAsStopped() {
    assertFalse(Swerve.isZeroCommand(new ChassisSpeeds(0.01, 0.0, 0.0)));
  }

  @Test
  void invalidGyroReadingsAreRejected() {
    assertNull(Swerve.headingFromSignal(false, 45.0));
    assertNull(Swerve.headingFromSignal(true, Double.NaN));
  }

  @Test
  void fieldRelativeSpeedsRequireAValidHeading() {
    assertNull(Swerve.toRobotRelativeSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0), null));
  }

  @Test
  void fieldRelativeSpeedsAreConvertedUsingHeading() {
    ChassisSpeeds robotRelativeSpeeds =
        Swerve.toRobotRelativeSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0), Rotation2d.fromDegrees(90.0));

    assertTrue(Math.abs(robotRelativeSpeeds.vxMetersPerSecond) < 1e-9);
    assertTrue(Math.abs(robotRelativeSpeeds.vyMetersPerSecond + 1.0) < 1e-9);
  }

}
