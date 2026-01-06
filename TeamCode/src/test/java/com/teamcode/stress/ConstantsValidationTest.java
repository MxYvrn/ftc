package com.teamcode.stress;

import com.teamcode.Constants;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Validation tests for Constants class.
 * Tests critical bugs in unit conversion constants.
 */
public class ConstantsValidationTest {

    /**
     * BUG-001 FIXED: Test that RPM to TPS conversion is mathematically correct.
     * RPM (revolutions per minute) * ticks_per_rev / 60 = ticks per second
     */
    @Test
    public void testRPMToTPSConversionCorrectness() {
        double ticksPerRev = Constants.SHOOTER_TICKS_PER_REV;
        double expectedConversionFactor = ticksPerRev / 60.0;
        double actualConversionFactor = Constants.SHOOTER_RPM_TO_TPS;
        
        assertEquals("SHOOTER_RPM_TO_TPS must divide ticks_per_rev by 60 to convert RPM to TPS",
                expectedConversionFactor, actualConversionFactor, 0.001);
    }

    /**
     * Test that RPM conversion works correctly with actual values.
     */
    @Test
    public void testRPMToTPSWithActualValues() {
        // Test with LOW speed: 200 RPM
        double rpm = Constants.SHOOTER_SPEED_LOW;
        double expectedTPS = rpm * (Constants.SHOOTER_TICKS_PER_REV / 60.0);
        double calculatedTPS = rpm * Constants.SHOOTER_RPM_TO_TPS;
        
        assertEquals("300 RPM should convert to correct TPS",
                expectedTPS, calculatedTPS, 0.001);
        
        // Verify reasonable range (300 RPM should be ~1922 TPS, not 115,350)
        assertTrue("TPS should be reasonable (not 60x too high)",
                calculatedTPS < 5000); // Sanity check
    }

    /**
     * Test all speed modes have reasonable TPS values.
     */
    @Test
    public void testAllSpeedModesHaveReasonableTPS() {
        double[] rpms = {
            Constants.SHOOTER_SPEED_LOW,
            Constants.SHOOTER_SPEED_MEDIUM,
            Constants.SHOOTER_SPEED_MAX
        };
        
        for (double rpm : rpms) {
            double tps = rpm * Constants.SHOOTER_RPM_TO_TPS;
            // Sanity check: TPS should be roughly RPM * 6.4 (for 384.5 ticks/rev)
            // Not RPM * 384.5 (which would be 60x too high)
            double expectedApprox = rpm * 6.4;
            assertTrue(String.format("TPS for %.0f RPM should be reasonable (got %.1f, expected ~%.1f)",
                    rpm, tps, expectedApprox),
                    Math.abs(tps - expectedApprox) < expectedApprox * 0.5); // 50% tolerance
        }
    }

    /**
     * Test that constants are positive and non-zero.
     */
    @Test
    public void testConstantsArePositive() {
        assertTrue("TICKS_PER_REV must be positive", Constants.SHOOTER_TICKS_PER_REV > 0);
        assertTrue("RPM_TO_TPS must be positive", Constants.SHOOTER_RPM_TO_TPS > 0);
        assertTrue("SPEED_LOW must be positive", Constants.SHOOTER_SPEED_LOW > 0);
        assertTrue("SPEED_MEDIUM must be positive", Constants.SHOOTER_SPEED_MEDIUM > 0);
        assertTrue("SPEED_MAX must be positive", Constants.SHOOTER_SPEED_MAX > 0);
    }
}

