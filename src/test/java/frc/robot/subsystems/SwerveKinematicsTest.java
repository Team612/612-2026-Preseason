package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

// not my code

class SwerveKinematicsTest {
  @Test
  void combinedMotionIsDiscretizedWithoutChangingTranslationRotationRatioBeforeDiscretization()
      throws Exception {
    var wheelStates = Swerve.class.getDeclaredMethod("getWheelStates", ChassisSpeeds.class);
    wheelStates.setAccessible(true);
    for (double vx : new double[] {-3, -0.5, 0, 0.5, 3}) {
      for (double vy : new double[] {-3, 0, 3}) {
        for (double omega : new double[] {-6, -1, 0, 1, 6}) {
          var request = new ChassisSpeeds(vx, vy, omega);
          var states = (SwerveModuleState[]) wheelStates.invoke(null, request);
          for (var state : states) {
            assertTrue(Math.abs(state.speedMetersPerSecond)
                <= Constants.SwerveConstants.kMaxSpeedMetersPerSecond + 1e-9);
          }
          var actual = Constants.swerveKinematics.toChassisSpeeds(states);
          double scale = omega != 0 ? actual.omegaRadiansPerSecond / omega
              : vx != 0 ? actual.vxMetersPerSecond / vx
              : vy != 0 ? actual.vyMetersPerSecond / vy : 1;
          assertTrue(scale > 0 && scale <= 1 + 1e-9);
          var expected = ChassisSpeeds.discretize(request.times(scale), Constants.kLoopPeriodSeconds);
          assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 1e-9);
          assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 1e-9);
          assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 1e-9);
        }
      }
    }
  }
}
