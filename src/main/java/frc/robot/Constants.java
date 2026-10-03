package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Contains values that remain constant (unless you change them) and are used
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double trackWidth = 0.550;
  public static final double wheelBase = 0.555;

  public static final SwerveDriveKinematics swerveKinematics =
   new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
  );

  public static final double xPercent = 0.2;
  public static final double yPercent = 0.2;
  public static final double zPercent = 0.2;

  public static final double zOFFset =(zPercent)/(Math.sqrt((trackWidth/2)*(trackWidth/2)+(wheelBase/2)*(wheelBase/2)));


  public static final double kp = 0.5;

  public static final double DEADBAND = 0.05;

  public static final int gyroID = 0;

  public static final double frontLEncoderOffset = 0.277;
  public static final int frontLSteerMotorID = 1;
  public static final int frontLDriveMotorID = 2;
  public static final int frontLCANcoderID = 3;

  public static final double frontREncoderOffset = 0.398;
  public static final int frontRSteerMotorID = 4;
  public static final int frontRDriveMotorID = 3;
  public static final int frontRCANcoderID = 2;

  public static final double backLEncoderOffset =-0.260;
  public static final int backLSteerMotorID = 8;
  public static final int backLDriveMotorID = 7;
  public static final int backLCANcoderID = 4;

  public static final double backREncoderOffset = 0.022;
  public static final int backRSteerMotorID = 6;
  public static final int backRDriveMotorID = 5;
  public static final int backRCANcoderID = 1;

  public static final int controllerPOrtNumber = 0;
}
