package com.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.teamcode.Constants;

/**
 * Subsystem for managing continuous rotation servos.
 * These servos run continuously at a set speed (like motors).
 */
public class CRServoSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;

    private double servo1Power = Constants.CR_SERVO_SPEED;
    private double servo2Power = Constants.CR_SERVO_SPEED;

    public CRServoSubsystem(HardwareMap hw) {
        servo1 = hw.get(CRServo.class, Constants.CR_SERVO_1_NAME);
        servo2 = hw.get(CRServo.class, Constants.CR_SERVO_2_NAME);

        // Start servos immediately
        start();
    }

    /**
     * Start both servos at configured speed.
     */
    public void start() {
        servo1.setPower(servo1Power);
        servo2.setPower(servo2Power);
    }

    /**
     * Stop both servos.
     */
    public void stop() {
        servo1.setPower(0.5); // 0.5 = stopped for CR servos
        servo2.setPower(0.5);
    }

    /**
     * Set custom power for servo 1.
     * @param power 0.0 (full reverse) to 1.0 (full forward), 0.5 = stopped
     */
    public void setServo1Power(double power) {
        servo1Power = power;
        servo1.setPower(power);
    }

    /**
     * Set custom power for servo 2.
     * @param power 0.0 (full reverse) to 1.0 (full forward), 0.5 = stopped
     */
    public void setServo2Power(double power) {
        servo2Power = power;
        servo2.setPower(power);
    }

    /**
     * Get current power setting for servo 1.
     */
    public double getServo1Power() {
        return servo1Power;
    }

    /**
     * Get current power setting for servo 2.
     */
    public double getServo2Power() {
        return servo2Power;
    }
}
