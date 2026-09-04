package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(
              Constants.SwerveConstants.kWheelBaseMeters / 2.0,
              Constants.SwerveConstants.kTrackWidthMeters / 2.0),
          new Translation2d(
              Constants.SwerveConstants.kWheelBaseMeters / 2.0,
              -Constants.SwerveConstants.kTrackWidthMeters / 2.0),
          new Translation2d(
              -Constants.SwerveConstants.kWheelBaseMeters / 2.0,
              Constants.SwerveConstants.kTrackWidthMeters / 2.0),
          new Translation2d(
              -Constants.SwerveConstants.kWheelBaseMeters / 2.0,
              -Constants.SwerveConstants.kTrackWidthMeters / 2.0));
  private final SwerveModule[] modules = {
    new SwerveModule("FrontLeft", Constants.SwerveConstants.kFrontLeft),
    new SwerveModule("FrontRight", Constants.SwerveConstants.kFrontRight),
    new SwerveModule("RearLeft", Constants.SwerveConstants.kRearLeft),
    new SwerveModule("RearRight", Constants.SwerveConstants.kRearRight)
  };
  private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
  private int telemetryCycles;

  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    if (!validGeometry()
        || !Double.isFinite(robotRelativeSpeeds.vxMetersPerSecond)
        || !Double.isFinite(robotRelativeSpeeds.vyMetersPerSecond)
        || !Double.isFinite(robotRelativeSpeeds.omegaRadiansPerSecond)) {
      stop();
      return;
    }
    commandedSpeeds = robotRelativeSpeeds;
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        targetStates, Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
    for (int index = 0; index < modules.length; index++) {
      modules[index].setState(targetStates[index]);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int index = 0; index < modules.length; index++) {
      states[index] = modules[index].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int index = 0; index < modules.length; index++) {
      positions[index] = modules[index].getPosition();
    }
    return positions;
  }

  public boolean hasValidModulePositions() {
    for (SwerveModule module : modules) {
      if (!module.hasValidDrivePosition()) {
        return false;
      }
    }
    return true;
  }

  public void stop() {
    commandedSpeeds = new ChassisSpeeds();
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  private boolean validGeometry() {
    return Constants.SwerveConstants.kWheelBaseMeters > 0.0
        && Constants.SwerveConstants.kTrackWidthMeters > 0.0
        && Constants.SwerveConstants.kMaxSpeedMetersPerSecond > 0.0
        && Double.isFinite(Constants.SwerveConstants.kWheelBaseMeters)
        && Double.isFinite(Constants.SwerveConstants.kTrackWidthMeters)
        && Double.isFinite(Constants.SwerveConstants.kMaxSpeedMetersPerSecond);
  }

  @Override
  public void periodic() {
    telemetryCycles++;
    if (telemetryCycles % 5 != 0) {
      return;
    }
    SmartDashboard.putNumber("Swerve/CommandedVxMetersPerSecond", commandedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedVyMetersPerSecond", commandedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/CommandedOmegaRadiansPerSecond", commandedSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/BatteryVoltage", RobotController.getBatteryVoltage());
    for (SwerveModule module : modules) {
      module.publishTelemetry();
    }
  }
}
