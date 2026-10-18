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
    drive.getSimState().setSupplyVoltage(12.0);
    turn.getSimState().setSupplyVoltage(12.0);
    encoder.getSimState().setSupplyVoltage(12.0);
    drive.getSimState().setRawRotorPosition(0.0);
    drive.getSimState().setRotorVelocity(0.0);
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
