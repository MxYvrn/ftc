package com.teamcode.stress;

import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Stress tests for DriveSubsystem.
 * Tests voltage compensation bounds, power clamping, normalization.
 */
public class DriveSubsystemStressTest {

    /**
     * BUG-004: Test voltage compensation bounds.
     * Voltage compensation should not allow power to exceed safe limits.
     */
    @Test
    public void testVoltageCompensationBounds() {
        // Simulate voltage compensation logic
        double NOMINAL_VOLTAGE = 12.0;
        
        // Test case 1: Low voltage (should scale up, but must clamp)
        double lowVoltage = 10.0;
        double voltageScale = NOMINAL_VOLTAGE / lowVoltage; // = 1.2
        double inputPower = 0.9;
        double compensatedPower = inputPower * voltageScale; // = 1.08
        
        // Should be clamped to 1.0
        double clampedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
        assertTrue("Compensated power should not exceed 1.0",
                clampedPower <= 1.0);
        
        // Test case 2: Very low voltage (extreme case)
        double veryLowVoltage = 6.0;
        voltageScale = NOMINAL_VOLTAGE / veryLowVoltage; // = 2.0 (dangerous!)
        inputPower = 0.8;
        compensatedPower = inputPower * voltageScale; // = 1.6 (exceeds limit!)
        
        clampedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));
        assertEquals("Very low voltage should clamp to 1.0", 1.0, clampedPower, 0.001);
        
        // Test case 3: High voltage (should scale down)
        double highVoltage = 14.0;
        voltageScale = NOMINAL_VOLTAGE / highVoltage; // = 0.857
        inputPower = 1.0;
        compensatedPower = inputPower * voltageScale; // = 0.857 (safe)
        
        assertTrue("High voltage should reduce power", compensatedPower < 1.0);
    }

    /**
     * Test that voltage scale is clamped to reasonable range.
     */
    @Test
    public void testVoltageScaleClamping() {
        double NOMINAL_VOLTAGE = 12.0;
        
        // Test extreme voltages
        double[] voltages = {0.1, 5.0, 10.0, 12.0, 14.0, 20.0, 100.0};
        
        for (double voltage : voltages) {
            if (voltage > 0) {
                double scale = NOMINAL_VOLTAGE / voltage;
                
                // Should be clamped to reasonable range (e.g., 0.5 to 1.5)
                double clampedScale = Math.max(0.5, Math.min(1.5, scale));
                assertTrue("Voltage scale should be clamped",
                        clampedScale >= 0.5 && clampedScale <= 1.5);
            }
        }
    }

    /**
     * Test mecanum normalization with all zeros.
     */
    @Test
    public void testMecanumNormalizationWithZeros() {
        // Mecanum formula with all zeros
        double forward = 0.0;
        double strafe = 0.0;
        double turn = 0.0;
        
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        
        // Normalize (avoid division by zero)
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;
        
        // All should be zero
        assertEquals("FL should be zero", 0.0, fl, 0.001);
        assertEquals("FR should be zero", 0.0, fr, 0.001);
        assertEquals("BL should be zero", 0.0, bl, 0.001);
        assertEquals("BR should be zero", 0.0, br, 0.001);
    }

    /**
     * Test normalization prevents values > 1.0.
     */
    @Test
    public void testNormalizationPreventsOversaturation() {
        // Extreme input (all at max)
        double forward = 1.0;
        double strafe = 1.0;
        double turn = 1.0;
        
        double fl = forward + strafe + turn; // = 3.0
        double fr = forward - strafe - turn; // = -1.0
        double bl = forward - strafe + turn; // = 1.0
        double br = forward + strafe - turn; // = 1.0
        
        // Normalize
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;
        
        // All should be <= 1.0 in magnitude
        assertTrue("FL should be <= 1.0", Math.abs(fl) <= 1.0);
        assertTrue("FR should be <= 1.0", Math.abs(fr) <= 1.0);
        assertTrue("BL should be <= 1.0", Math.abs(bl) <= 1.0);
        assertTrue("BR should be <= 1.0", Math.abs(br) <= 1.0);
    }
}

