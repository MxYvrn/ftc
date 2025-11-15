package com.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Simple test OpMode to verify CR servos (rollers) are working.
 * Press START and the rollers should spin continuously.
 */
@TeleOp(name = "Test Rollers", group = "Test")
public class TestRollers extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftRoller = null;
        CRServo rightRoller = null;

        // Try to initialize servos
        try {
            leftRoller = hardwareMap.get(CRServo.class, "left_Roller");
            telemetry.addLine("✓ Left roller found");
        } catch (Exception e) {
            telemetry.addLine("✗ Left roller NOT FOUND");
        }

        try {
            rightRoller = hardwareMap.get(CRServo.class, "right_Roller");
            telemetry.addLine("✓ Right roller found");
        } catch (Exception e) {
            telemetry.addLine("✗ Right roller NOT FOUND");
        }

        telemetry.addLine();
        telemetry.addLine("Press START to spin rollers");
        telemetry.update();

        waitForStart();

        // Start the rollers spinning
        if (leftRoller != null) {
            leftRoller.setPower(1.0); // Full forward
            telemetry.addLine("Left roller: RUNNING at 1.0");
        } else {
            telemetry.addLine("Left roller: MISSING");
        }

        if (rightRoller != null) {
            rightRoller.setPower(0.0); // Full reverse (opposite direction)
            telemetry.addLine("Right roller: RUNNING at 0.0 (reverse)");
        } else {
            telemetry.addLine("Right roller: MISSING");
        }

        telemetry.addLine();
        telemetry.addLine("Rollers should be spinning now!");
        telemetry.addLine("Press STOP to end");
        telemetry.update();

        // Keep running until stopped
        while (opModeIsActive()) {
            idle();
        }

        // Stop rollers
        if (leftRoller != null) leftRoller.setPower(0.5);
        if (rightRoller != null) rightRoller.setPower(0.5);
    }
}
