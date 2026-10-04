package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class SwerveTest {
  @Test
  void currentCanMapIsAcceptedWhenAllIdsAreInRange() {
    assertTrue(Swerve.hasValidCanIds(new Constants.ModuleConfiguration[] {
      Constants.SwerveConstants.kFrontLeft,
      Constants.SwerveConstants.kFrontRight,
      Constants.SwerveConstants.kRearLeft,
      Constants.SwerveConstants.kRearRight
    }));
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
  void nonzeroChassisCommandIsNotTreatedAsStopped() {
    assertFalse(Swerve.isZeroCommand(new ChassisSpeeds(0.01, 0.0, 0.0)));
  }
}
