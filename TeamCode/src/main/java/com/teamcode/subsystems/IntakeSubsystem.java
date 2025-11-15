package com.teamcode.subsystems;

import static com.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple intake subsystem for ball collection.
 * Controls a single motor or roller mechanism.
 * Gracefully handles missing motor (intake disconnected/misconfigured).
 */
public class IntakeSubsystem {
    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hw, String motorName) {
        DcMotor motor = null;
        try {
            motor = hw.get(DcMotor.class, motorName);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            // Intake motor not found in config
            motor = null;
        }
        intakeMotor = motor;
    }

    /**
     * @return True if intake motor is present and functional
     */
    public boolean isPresent() {
        return intakeMotor != null;
    }

    /**
     * Turn intake on (collecting).
     */
    public void intakeOn() {
        if (intakeMotor != null) {
            intakeMotor.setPower(INTAKE_POWER_COLLECT);
        }
    }

    /**
     * Turn intake off.
     */
    public void intakeOff() {
        if (intakeMotor != null) {
            intakeMotor.setPower(0);
        }
    }

    /**
     * Reverse intake (eject).
     */
    public void reverse() {
        if (intakeMotor != null) {
            intakeMotor.setPower(INTAKE_POWER_EJECT);
        }
    }

    /**
     * Set custom intake power.
     */
    public void setPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }
}
