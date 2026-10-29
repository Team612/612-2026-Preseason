package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.reflect.Field;
import org.junit.jupiter.api.*;

class SwerveCommandTest {
  private static Swerve swerve;
  private static SwerveModule[] modules;
  private static final TalonFX[] drives = new TalonFX[4];
  private static final TalonFX[] turns = new TalonFX[4];
  private static final CANcoder[] encoders = new CANcoder[4];
  private static Pigeon2 gyro;
  private static double simulatedYaw;

  private static <T> T field(Object object, String name, Class<T> type) throws Exception {
    Field field = object.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return type.cast(field.get(object));
  }

  @BeforeAll
  static void setup() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    swerve = new Swerve();
    modules = field(swerve, "modules", SwerveModule[].class);
    gyro = field(swerve, "gyro", Pigeon2.class);
    for (int i = 0; i < 4; i++) {
      drives[i] = field(modules[i], "driveMotor", TalonFX.class);
      turns[i] = field(modules[i], "turnMotor", TalonFX.class);
      encoders[i] = field(modules[i], "encoder", CANcoder.class);
      drives[i].setSafetyEnabled(false);
      turns[i].setSafetyEnabled(false);
    }
    for (int i = 0; i < 80; i++) tick();
    assertTrue(swerve.isReady());
  }

  private static void tick() throws Exception {
    gyro.getSimState().setSupplyVoltage(12);
    gyro.getSimState().setRawYaw(simulatedYaw);
    for (int i = 0; i < 4; i++) {
      var config = field(modules[i], "configuration", Constants.ModuleConfiguration.class);
      drives[i].getSimState().setSupplyVoltage(12);
      drives[i].getSimState().setRawRotorPosition(0);
      drives[i].getSimState().setRotorVelocity(0);
      turns[i].getSimState().setSupplyVoltage(12);
      turns[i].getSimState().setRawRotorPosition(0);
      encoders[i].getSimState().setSupplyVoltage(12);
      encoders[i].getSimState().setRawPosition(
          config.encoderOffsetRotations() * (config.encoderInverted() ? -1 : 1));
      encoders[i].getSimState().setVelocity(0);
    }
    Timer.delay(0.02);
    CommandScheduler.getInstance().run();
  }

  private static void mode(boolean enabled, boolean test) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setTest(test);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void reset() throws Exception {
    CommandScheduler.getInstance().cancelAll();
    swerve.stop();
    mode(false, false);
    simulatedYaw = 0;
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    tick();
    swerve.resetHeading();
  }

  @AfterAll
  static void close() {
    CommandScheduler.getInstance().unregisterSubsystem(swerve);
    for (int i = 0; i < 4; i++) {
      if (drives[i] != null) drives[i].close();
      if (turns[i] != null) turns[i].close();
      if (encoders[i] != null) encoders[i].close();
    }
    if (gyro != null) gyro.close();
    SignalLogger.stop();
  }

  @Test
  void fieldTranslationUsesHeadingAndAllianceWithoutChangingRotation() throws Exception {
    for (var alliance : new AllianceStationID[] {AllianceStationID.Blue1, AllianceStationID.Red1}) {
      mode(false, false);
      DriverStationSim.setAllianceStationId(alliance);
      DriverStationSim.notifyNewData();
      for (double yaw : new double[] {0, 90, -90, 180}) {
        simulatedYaw = yaw;
        for (int i = 0; i < 4; i++) tick();
        mode(true, false);
        assertTrue(swerve.driveOperatorRelative(new ChassisSpeeds(0.5, 0.2, 0.3)));
        var actual = field(swerve, "commandedSpeeds", ChassisSpeeds.class);
        double angle = Math.toRadians(yaw - (alliance == AllianceStationID.Red1 ? 180 : 0));
        assertEquals(0.5 * Math.cos(angle) + 0.2 * Math.sin(angle), actual.vxMetersPerSecond, 1e-6);
        assertEquals(-0.5 * Math.sin(angle) + 0.2 * Math.cos(angle), actual.vyMetersPerSecond, 1e-6);
        assertEquals(0.3, actual.omegaRadiansPerSecond, 1e-6);
        mode(false, false);
      }
    }
  }

  @Test
  void toggleWhileMovingKeepsDemandContinuousAndDoesNotRequireNeutral() throws Exception {
    simulatedYaw = 90;
    for (int i = 0; i < 4; i++) tick();
    double[] forward = {0};
    var command = new SwerveDrive(swerve, () -> forward[0], () -> 0, () -> 0, () -> true);
    mode(true, false);
    CommandScheduler.getInstance().schedule(command);
    tick();
    forward[0] = 0.5;
    for (int i = 0; i < 12; i++) tick();
    var before = field(swerve, "commandedSpeeds", ChassisSpeeds.class);
    assertTrue(before.vyMetersPerSecond < -0.5);
    for (int i = 0; i < 6; i++) {
      command.toggleDriveMode();
      assertEquals(i % 2 != 0, SmartDashboard.getBoolean("Drive/FieldRelativeMode", true));
      tick();
      var after = field(swerve, "commandedSpeeds", ChassisSpeeds.class);
      assertFalse(SmartDashboard.getBoolean("Drive/WaitingForNeutral", true));
      assertTrue(Math.hypot(after.vxMetersPerSecond, after.vyMetersPerSecond) > 0.5);
      assertTrue(Math.hypot(after.vxMetersPerSecond - before.vxMetersPerSecond,
          after.vyMetersPerSecond - before.vyMetersPerSecond)
          <= Constants.SwerveConstants.kMaxSpeedMetersPerSecond
              * Constants.OperatorConstants.kTranslationReleaseSlewRatePerSecond
              * Constants.OperatorConstants.kMaxLimiterStepSeconds + 1e-6);
      before = after;
    }
  }

  @Test
  void smallRestingStickOffsetsArmPromptlyWithoutMotion() throws Exception {
    double[] axes = {0.04, 0.03, -0.04};
    var command = new SwerveDrive(swerve, () -> axes[0], () -> axes[1], () -> axes[2], () -> true);
    mode(true, false);
    CommandScheduler.getInstance().schedule(command);
    tick();
    assertFalse(SmartDashboard.getBoolean("Drive/WaitingForNeutral", true));
    assertEquals("Ready", SmartDashboard.getString("Drive/Status", ""));
    tick();
    assertEquals(0.0, field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond);
    axes[0] = 0.5;
    tick();
    assertTrue(field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond > 0.0);
  }

  @Test
  void heldStickAndReconnectStillRequireNeutral() throws Exception {
    double[] forward = {0.5};
    boolean[] connected = {true};
    var command = new SwerveDrive(swerve, () -> forward[0], () -> 0, () -> 0, () -> connected[0]);
    mode(true, false);
    CommandScheduler.getInstance().schedule(command);
    tick();
    assertTrue(SmartDashboard.getBoolean("Drive/WaitingForNeutral", false));
    assertEquals("Center sticks to arm drive", SmartDashboard.getString("Drive/Status", ""));
    assertEquals(0.0, field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond);
    forward[0] = 0;
    tick();
    forward[0] = 0.5;
    tick();
    assertTrue(field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond > 0.0);
    connected[0] = false;
    tick();
    assertEquals("Controller disconnected", SmartDashboard.getString("Drive/Status", ""));
    connected[0] = true;
    tick();
    assertTrue(SmartDashboard.getBoolean("Drive/WaitingForNeutral", false));
    assertEquals(0.0, field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond);
    forward[0] = 0;
    tick();
    assertFalse(SmartDashboard.getBoolean("Drive/WaitingForNeutral", true));
  }

  @Test
  void sysIdRunsThroughSchedulerAndCancellationStopsAllModules() throws Exception {
    mode(true, true);
    tick();
    Command command = swerve.sysId(false, true, SysIdRoutine.Direction.kForward);
    assertTrue(command.getRequirements().contains(swerve));
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < 12; i++) tick();
    assertTrue(command.isScheduled());
    for (SwerveModule module : modules) {
      assertEquals(2.0, field(module, "driveVolts", Double.class));
    }
    command.cancel();
    for (SwerveModule module : modules) {
      assertEquals(0.0, field(module, "driveVolts", Double.class));
      assertEquals(0.0, field(module, "turnVolts", Double.class));
    }
  }

  @Test
  void sysIdCannotRunInTeleopAndStopsWhenLeavingTest() throws Exception {
    mode(true, false);
    tick();
    Command rejected = swerve.sysId(true, true, SysIdRoutine.Direction.kForward);
    CommandScheduler.getInstance().schedule(rejected);
    tick();
    assertFalse(rejected.isScheduled());
    for (SwerveModule module : modules) {
      assertEquals(0.0, field(module, "turnVolts", Double.class));
    }
    mode(true, true);
    Command command = swerve.sysId(true, true, SysIdRoutine.Direction.kReverse);
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < 10; i++) tick();
    assertTrue(command.isScheduled());
    for (SwerveModule module : modules) {
      assertEquals(-2.0, field(module, "turnVolts", Double.class));
    }
    mode(true, false);
    tick();
    assertFalse(command.isScheduled());
    for (SwerveModule module : modules) {
      assertEquals(0.0, field(module, "turnVolts", Double.class));
    }
  }

  @Test
  void redPerspectiveAndHeadingResetHaveConsistentDriverForward() throws Exception {
    mode(false, false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    tick();
    mode(true, false);
    assertTrue(swerve.driveOperatorRelative(new ChassisSpeeds(0.5, 0, 0)));
    assertEquals(-0.5, field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond, 1e-6);
    swerve.stop();
    swerve.resetHeading();
    tick();
    assertTrue(swerve.driveOperatorRelative(new ChassisSpeeds(0.5, 0, 0)));
    assertEquals(0.5, field(swerve, "commandedSpeeds", ChassisSpeeds.class).vxMetersPerSecond, 1e-6);
    assertEquals(180.0, Math.abs(swerve.getPose().getRotation().getDegrees()), 1e-6);
  }
}
