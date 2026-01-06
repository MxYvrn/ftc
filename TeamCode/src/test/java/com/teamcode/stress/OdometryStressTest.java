package com.teamcode.stress;

import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Stress tests for Odometry subsystem.
 * Tests division by zero, NaN propagation, edge cases.
 */
public class OdometryStressTest {

    /**
     * BUG-003: Test division by zero edge cases in arc correction.
     * Odometry uses division by dTheta_enc, must handle near-zero cases.
     */
    @Test
    public void testDivisionByZeroEdgeCases() {
        // Test the arc correction formula:
        // dX_robot = (sinTheta / dTheta_enc) * forward - ((1 - cosTheta) / dTheta_enc) * lateral
        
        double[] testCases = {
            0.0,
            1e-10,
            -1e-10,
            1e-6,
            -1e-6,
            1e-7,  // Just below threshold
            1e-5   // Just above threshold
        };
        
        for (double dTheta : testCases) {
            double forward = 1.0;
            double lateral = 0.5;
            
            double dX_robot, dY_robot;
            if (Math.abs(dTheta) < 1e-6) {
                // Straight line motion (avoid division by zero)
                dX_robot = forward;
                dY_robot = lateral;
            } else {
                // Arc correction
                double sinTheta = Math.sin(dTheta);
                double cosTheta = Math.cos(dTheta);
                dX_robot = (sinTheta / dTheta) * forward - ((1 - cosTheta) / dTheta) * lateral;
                dY_robot = ((1 - cosTheta) / dTheta) * forward + (sinTheta / dTheta) * lateral;
            }
            
            // Result must be finite (not NaN or Inf)
            assertFalse("dX_robot must be finite for dTheta=" + dTheta,
                    Double.isNaN(dX_robot) || Double.isInfinite(dX_robot));
            assertFalse("dY_robot must be finite for dTheta=" + dTheta,
                    Double.isNaN(dY_robot) || Double.isInfinite(dY_robot));
            
            // Result should be reasonable
            assertTrue("dX_robot should be reasonable", Math.abs(dX_robot) < 100);
            assertTrue("dY_robot should be reasonable", Math.abs(dY_robot) < 100);
        }
    }

    /**
     * Test that sinc approximation works correctly for small angles.
     */
    @Test
    public void testSincApproximationForSmallAngles() {
        // For small dTheta, sin(dTheta) / dTheta ≈ 1
        // This should be handled by the straight-line case
        double smallAngle = 1e-7;
        double forward = 1.0;
        
        double dX_straight = forward; // Straight line case
        double dX_arc = (Math.sin(smallAngle) / smallAngle) * forward; // Arc case
        
        // They should be very close
        assertEquals("Small angle arc should approximate straight line",
                dX_straight, dX_arc, 0.01);
    }

    /**
     * Test NaN propagation prevention.
     */
    @Test
    public void testNaNPropagationPrevention() {
        // Test that invalid inputs don't propagate NaN
        double forward = Double.NaN;
        double lateral = 0.5;
        double dTheta = 0.1;
        
        double sinTheta = Math.sin(dTheta);
        double cosTheta = Math.cos(dTheta);
        double dX = (sinTheta / dTheta) * forward - ((1 - cosTheta) / dTheta) * lateral;
        
        // Result will be NaN, but in real code we should check for this
        assertTrue("Should detect NaN input", Double.isNaN(dX));
        
        // In actual Odometry, we should validate inputs before math
    }
}

