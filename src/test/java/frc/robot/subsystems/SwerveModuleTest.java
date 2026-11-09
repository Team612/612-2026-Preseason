package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants;
import java.lang.reflect.Field;
import org.junit.jupiter.api.*;

class SwerveModuleTest {
  private static SwerveModule module;
  private static TalonFX drive;
  private static TalonFX turn;
  private static CANcoder encoder;

  private static <T> T field(String name, Class<T> type) throws Exception {
    Field field = SwerveModule.class.getDeclaredField(name);
    field.setAccessible(true);
    return type.cast(field.get(module));
  }

  @BeforeAll
  static void setupHardware() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    module = new SwerveModule("TestModule",
        new Constants.ModuleConfiguration(21, 22, 21, 0.125, false, false, false), true);
    drive = field("driveMotor", TalonFX.class);
    turn = field("turnMotor", TalonFX.class);
    encoder = field("encoder", CANcoder.class);
    drive.setSafetyEnabled(false);
    turn.setSafetyEnabled(false);
    for (int i = 0; i < 80; i++) {
      updateSensors();
      module.periodic();
    }
    assertTrue(module.isReady(), "Simulated CAN feedback should become ready");
  }

  private static void updateSensors() {
    updateSensors(12.0);
  }

  private static void updateSensors(double supplyVolts) {
    updateSensors(supplyVolts, 0.0);
  }

  private static void updateSensors(double supplyVolts, double measuredSpeed) {
    drive.getSimState().setSupplyVoltage(supplyVolts);
    turn.getSimState().setSupplyVoltage(supplyVolts);
    encoder.getSimState().setSupplyVoltage(supplyVolts);
    drive.getSimState().setRawRotorPosition(0.0);
    drive.getSimState().setRotorVelocity(measuredSpeed * Constants.SwerveConstants.kDriveReduction
        / (Math.PI * Constants.SwerveConstants.kWheelDiameterMeters));
    turn.getSimState().setRawRotorPosition(0.0);
    encoder.getSimState().setRawPosition(0.125);
    encoder.getSimState().setVelocity(0.0);
    Timer.delay(0.02);
  }

  private static void mode(boolean enabled, boolean test) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setTest(test);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    updateSensors();
  }

  @AfterEach
  void stop() {
    module.stop();
    mode(false, false);
  }

  @AfterAll
  static void closeHardware() {
    if (drive != null) drive.close();
    if (turn != null) turn.close();
    if (encoder != null) encoder.close();
  }

  @Test
  void pointingAtZeroSpeedActuallySteersAndAppliesOffsetOnce() throws Exception {
    mode(true, false);
    assertEquals(0.0, module.getState().angle.getRadians(), 1e-6);
    for (int i = 0; i < 10; i++) {
      updateSensors();
      assertTrue(module.pointAt(Rotation2d.fromDegrees(45)));
    }
    assertEquals(0.0, field("driveVolts", Double.class));
    assertTrue(field("turnVolts", Double.class) > 0.0);
    assertEquals(45.0, module.getTargetState().angle.getDegrees(), 1e-6);
    assertEquals(0.0, module.getTargetState().speedMetersPerSecond);
  }

  @Test
  void characterizationRequiresTestModeAndRespectsCaps() throws Exception {
    mode(true, false);
    assertFalse(module.characterize(false, 2.0));
    assertEquals(0.0, field("driveVolts", Double.class));
    mode(true, true);
    assertTrue(module.characterize(false, 20.0));
    assertEquals(Constants.SwerveConstants.kMaxDriveVolts, field("driveVolts", Double.class));
    assertTrue(module.characterize(true, -20.0));
    assertEquals(0.0, field("driveVolts", Double.class));
    assertEquals(-Constants.SwerveConstants.kMaxTurnVolts, field("turnVolts", Double.class));
    assertFalse(module.characterize(true, Double.NaN));
    assertEquals(0.0, field("turnVolts", Double.class));
  }

  @Test
  void steadyDemandDoesNotToggleDriveCommandBelowTwelveVolts() throws Exception {
    mode(true, false);
    // Fixed encoder feedback isolates voltage gating in our control code.
    // This is not a battery/current-limit or mechanical drivetrain simulation.
    for (int i = 0; i < 10; i++) { // yay for loop
      updateSensors();
      assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
    }
    for (double supplyVolts : new double[] {12.5, 11.9, 11.75, 11.5, 10.5, 11.9}) {
      for (int i = 0; i < 10; i++) {
        updateSensors(supplyVolts);
        assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
        assertEquals(Constants.SwerveConstants.kDriveKV
            + Math.min(Constants.SwerveConstants.kDriveP,
                Constants.SwerveConstants.kMaxDriveCorrectionVolts),
            field("driveVolts", Double.class), 1e-6);
      }
    }
  }

  @Test
  void changingSpeedFeedbackCannotStepDriveVoltageAndStopIsImmediate() throws Exception {
    mode(true, false);
    double previous = 0;
    for (int i = 0; i < 60; i++) {
      // Exercise lagging and overshooting feedback as supply falls through 11.75 V (guess)
      updateSensors(i < 20 ? 12.5 : 11.5, (i / 10) % 2 == 0 ? 0.0 : 2.0);
      assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
      double next = field("driveVolts", Double.class);
      if (Constants.SwerveConstants.kDriveVoltageSlewRateVoltsPerSecond == 0.0) {
        assertEquals(field("requestedDriveVolts", Double.class), next, 1e-9);
      } else {
        assertTrue(Math.abs(next - previous)
            <= Constants.SwerveConstants.kDriveVoltageSlewRateVoltsPerSecond
                * Constants.SwerveConstants.kDriveVoltageMaxStepSeconds + 1e-9);
      }
      previous = next;
    }
    module.stop();
    assertEquals(0.0, field("driveVolts", Double.class));
    assertEquals(0.0, field("turnVolts", Double.class));
    updateSensors();
    assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
    double requested = field("requestedDriveVolts", Double.class);
    double firstStep = Constants.SwerveConstants.kDriveVoltageSlewRateVoltsPerSecond
        * Constants.SwerveConstants.kDriveVoltageMaxStepSeconds;
    double restarted = field("driveVolts", Double.class);
    assertTrue(restarted > 0.0);
    assertTrue(restarted <= (firstStep == 0.0 ? requested : Math.min(requested, firstStep)) + 1e-9);
  }

  @Test
  void moduleRestartsVoltageAfterWatchdogGapWithoutTeleopCommand() throws Exception {
    mode(true, false);
    module.stop();
    for (int i = 0; i < 5; i++) {
      updateSensors();
      assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
    }
    assertTrue(field("driveVolts", Double.class) > 0.0);
    Timer.delay(Constants.SwerveConstants.kMotorSafetyExpirationSeconds);
    updateSensors();
    assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
    assertEquals(0.0, field("driveVolts", Double.class));
    updateSensors();
    assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
    assertTrue(field("driveVolts", Double.class) > 0.0);
    assertTrue(field("driveVolts", Double.class)
        <= Constants.SwerveConstants.kDriveVoltageSlewRateVoltsPerSecond
            * Constants.SwerveConstants.kDriveVoltageMaxStepSeconds + 1e-9);
  }

  @Test
  void optimizationReversesDriveAndPerpendicularDemandDoesNotDriveSideways() throws Exception {
    mode(true, false);
    for (int i = 0; i < 10; i++) {
      updateSensors();
      assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(170))));
    }
    assertEquals(-10, module.getTargetState().angle.getDegrees(), 1e-6);
    assertEquals(-Math.cos(Math.toRadians(10)), module.getTargetState().speedMetersPerSecond, 1e-6);
    assertTrue(field("driveVolts", Double.class) < 0);
    updateSensors();
    assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(90))));
    assertEquals(0.0, field("driveVolts", Double.class), 1e-6);
  }

  @Test
  void disabledAndInvalidRequestsStopBothMotors() throws Exception {
    mode(true, false);
    assertTrue(module.setState(new SwerveModuleState(1.0, Rotation2d.kZero)));
    assertTrue(field("driveVolts", Double.class) > 0.0);
    assertFalse(module.pointAt(null));
    assertEquals(0.0, field("driveVolts", Double.class));
    assertEquals(0.0, field("turnVolts", Double.class));
    mode(false, false);
    assertFalse(module.pointAt(Rotation2d.fromDegrees(90)));
    assertFalse(module.characterize(false, 2.0));
    assertEquals(0.0, field("driveVolts", Double.class));
    assertEquals(0.0, field("turnVolts", Double.class));
  }
}
