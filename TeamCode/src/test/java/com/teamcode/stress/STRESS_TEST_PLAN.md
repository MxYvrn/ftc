# Stress Test Plan

## Test Framework
- **Language**: Java
- **Test Runner**: JUnit 4 (already configured in build.gradle)
- **Mocking**: Manual mocks (FTC SDK doesn't support standard mocking frameworks easily)

## High-Risk Areas Identified

### 1. ShooterSubsystem (CRITICAL)
- **Risk**: Incorrect RPM calculation, wrong velocity sign
- **Tests**:
  - RPM to TPS conversion accuracy
  - Velocity sign consistency
  - Enabled/disabled state transitions
  - Edge cases: 0 RPM, negative RPM, very high RPM

### 2. Odometry (HIGH)
- **Risk**: Division by zero, NaN propagation, encoder failures
- **Tests**:
  - Division by zero in arc correction
  - NaN/Inf handling
  - Missing encoder graceful degradation
  - IMU failure recovery
  - Very large dt (loop stall)

### 3. DriveSubsystem (HIGH)
- **Risk**: Power clamping, voltage compensation bounds, null hardware
- **Tests**:
  - Voltage compensation bounds (low/high voltage)
  - Power clamping edge cases
  - Missing motor handling
  - Normalization with all zeros

### 4. TeleOpMain Loop (MEDIUM)
- **Risk**: Race conditions, button edge detection failures, telemetry overflow
- **Tests**:
  - Button edge detection rapid toggling
  - Telemetry rate limiting
  - System.nanoTime() overflow handling

### 5. FeederSubsystem (MEDIUM)
- **Risk**: Ramp timer reset logic, RT coupling edge cases
- **Tests**:
  - Ramp timer reset on rapid toggling
  - RT coupling with various intake states
  - Power calculation bounds

### 6. IntakeSubsystem (MEDIUM)
- **Risk**: Motor disconnect mid-match, direction sign calculation
- **Tests**:
  - Direction sign after motor null
  - Power caching correctness
  - Outtake toggle state management

## Test Categories

### Unit Tests (Isolated Components)
- Mock hardware dependencies
- Test pure logic/math
- Fast execution (< 1ms each)

### Integration Tests (Subsystem Interaction)
- Mock HardwareMap with multiple devices
- Test subsystem interactions
- Test state transitions

### Stress Tests (Load/Edge Cases)
- Repeated rapid calls
- Boundary value testing
- Resource exhaustion scenarios

### Regression Tests (Previously Fixed Bugs)
- Capture known failures
- Prevent regressions

## Test Execution Strategy

1. **Fast feedback**: Run unit tests on every build
2. **Pre-commit**: Run all tests including stress tests
3. **Nightly**: Extended stress tests (1000+ iterations)
4. **Hardware-in-loop**: Manual tests on actual robot (separate)

## Success Criteria

- All critical bugs must have tests that reproduce the issue
- Tests must be deterministic (no flakiness)
- Tests must run quickly (< 5 seconds total for unit tests)
- 100% code coverage for critical paths is NOT required, but all bug fixes must have tests

