package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class DriveVoltageSlewTest {
  private double now;

  private DriveVoltageLimiter limiter(double rate) {
    return new DriveVoltageLimiter(rate, 0.04, 0.1, () -> now);
  }

  private double step(DriveVoltageLimiter limiter, double target, double seconds) {
    now += seconds;
    return limiter.calculate(target);
  }

  @Test
  void zeroDisablesRampInBothDirections() {
    var limiter = limiter(0);
    assertEquals(6, step(limiter, 6, 0.02));
    assertEquals(-6, step(limiter, -6, 0.02));
  }

  @Test
  void elapsedTimeControlsTheRampAndRepeatedCallsCannotAccelerateIt() {
    var limiter = limiter(24);
    assertEquals(0.24, step(limiter, 6, 0.01), 1e-9);
    assertEquals(0.24, step(limiter, 6, 0), 1e-9);
    assertEquals(0.96, step(limiter, 6, 0.03), 1e-9);
    assertEquals(1.92, step(limiter, 6, 0.08), 1e-9);
  }

  @Test
  void higherRateRespondsFasterAndReversalsPassThroughZero() {
    var slow = limiter(24);
    var fast = limiter(48);
    now += 0.02;
    assertEquals(0.48, slow.calculate(6), 1e-9);
    assertEquals(0.96, fast.calculate(6), 1e-9);
    assertEquals(0.6, step(slow, 0.6, 0.02), 1e-9);
    assertEquals(0.12, step(slow, -6, 0.02), 1e-9);
    assertEquals(-0.36, step(slow, -6, 0.02), 1e-9);
    assertEquals(-0.2, step(slow, -0.2, 0.02), 1e-9);
    assertEquals(-0.2, step(slow, -0.2, 0.02), 1e-9);
  }

  @Test
  void normalZeroDemandRampsButExplicitResetIsImmediate() {
    var limiter = limiter(24);
    assertEquals(0.96, step(limiter, 6, 0.04), 1e-9);
    assertEquals(0.48, step(limiter, 0, 0.02), 1e-9);
    limiter.reset();
    assertEquals(0, limiter.calculate(6));
    assertEquals(0.48, step(limiter, 6, 0.02), 1e-9);
  }

  @Test
  void watchdogGapRestartsFromZeroEvenWhenRampIsDisabled() {
    for (double rate : new double[] {0, 24}) {
      var limiter = limiter(rate);
      step(limiter, 6, 0.04);
      assertEquals(0, step(limiter, 6, 0.11));
      assertEquals(rate == 0 ? 6 : 0.48, step(limiter, 6, 0.02), 1e-9);
    }
  }

  @Test
  void invalidInputsAndClockDiscontinuitiesClearHistory() {
    var limiter = limiter(24);
    step(limiter, 6, 0.02);
    assertEquals(0, step(limiter, Double.NaN, 0.02));
    assertEquals(0.48, step(limiter, 6, 0.02), 1e-9);
    assertEquals(0, step(limiter, 6, -0.01));
    now = Double.NaN;
    assertEquals(0, limiter.calculate(6));
    now = 1;
    assertEquals(0, limiter.calculate(6));
    assertEquals(0.48, step(limiter, 6, 0.02), 1e-9);
  }

  @Test
  void eachModuleHasIndependentHistoryAndBadConfigurationIsRejected() {
    var left = limiter(24);
    var right = limiter(24);
    now += 0.02;
    assertEquals(0.48, left.calculate(6), 1e-9);
    assertEquals(-0.48, right.calculate(-6), 1e-9);
    for (double rate : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertThrows(IllegalArgumentException.class, () -> limiter(rate));
    }
    assertThrows(IllegalArgumentException.class,
        () -> new DriveVoltageLimiter(24, 0.1, 0.1, () -> now));
  }
}
