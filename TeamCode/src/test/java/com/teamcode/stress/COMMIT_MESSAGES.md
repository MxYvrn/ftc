# Commit Messages for Bug Fixes

## BUG-001: Fix shooter RPM to TPS conversion (CRITICAL)

```
Fix: Correct SHOOTER_RPM_TO_TPS constant to divide by 60

The constant was missing division by 60, causing velocity setpoints
to be 60x too high. This prevented the shooter from reaching target
RPMs because the motor controller would saturate or fail.

Changed:
- Constants.java:219 - Added / 60.0 to RPM_TO_TPS conversion
- Added validation test to prevent regression

Test: ConstantsValidationTest.testRPMToTPSConversionCorrectness()
Impact: Shooter can now reach target RPMs correctly

Fixes: BUG-001
```

## BUG-002: Fix shooter velocity sign consistency (CRITICAL)

```
Fix: Remove negative velocity sign in ShooterSubsystem.update()

Motor direction was changed from REVERSE to FORWARD (user removed
setDirection call), but code still used negative velocity. This caused
the shooter to spin in the wrong direction or fail.

Changed:
- ShooterSubsystem.java:77 - Removed negative sign from setVelocity()
- Updated comment to reflect FORWARD direction

Test: ShooterSubsystemStressTest.testVelocitySignConsistency()
Impact: Shooter now spins in correct direction

Fixes: BUG-002
```

## BUG-004: Clamp voltage compensation scale (HIGH)

```
Fix: Add bounds clamping to voltage compensation scale

Voltage compensation could scale power beyond safe limits when battery
voltage was very low, potentially damaging motors.

Changed:
- DriveSubsystem.java:149-163 - Clamp voltageScale to [0.8, 1.2]
- Prevents dangerous over-compensation at extreme voltages

Test: DriveSubsystemStressTest.testVoltageCompensationBounds()
Impact: Motors protected from over-power at low battery voltage

Fixes: BUG-004
```

## BUG-007: Fix feeder ramp timer reset logic (MEDIUM)

```
Fix: Reset ramp timer whenever shoot command becomes active

Ramp timer only reset on rising edge, causing incorrect power levels
if shoot command toggled rapidly.

Changed:
- FeederSubsystem.java:31-41 - Reset timer when shootActive transitions to true
- More robust to rapid toggling

Test: FeederSubsystemStressTest.testRampTimerResetSequence()
Impact: Feeder ramps correctly on every activation

Fixes: BUG-007
```

