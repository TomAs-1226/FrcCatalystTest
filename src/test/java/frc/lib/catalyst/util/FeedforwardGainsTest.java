package frc.lib.catalyst.util;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for FeedforwardGains.
 * Tests elevator and arm feedforward calculations.
 */
class FeedforwardGainsTest {

    private static final double EPSILON = 1e-6;

    // ===== FACTORY METHODS =====

    @Test
    @DisplayName("simple: creates gains with kS and kV")
    void simple_createsGains() {
        FeedforwardGains ff = FeedforwardGains.simple(0.1, 0.5);
        assertEquals(0.1, ff.kS, EPSILON);
        assertEquals(0.5, ff.kV, EPSILON);
        assertEquals(0.0, ff.kA, EPSILON);
        assertEquals(0.0, ff.kG, EPSILON);
    }

    @Test
    @DisplayName("simple with kA: creates gains with kS, kV, kA")
    void simple_withKA_createsGains() {
        FeedforwardGains ff = FeedforwardGains.simple(0.1, 0.5, 0.02);
        assertEquals(0.1, ff.kS, EPSILON);
        assertEquals(0.5, ff.kV, EPSILON);
        assertEquals(0.02, ff.kA, EPSILON);
        assertEquals(0.0, ff.kG, EPSILON);
    }

    @Test
    @DisplayName("elevator: creates gains with gravity")
    void elevator_createsGainsWithGravity() {
        FeedforwardGains ff = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        assertEquals(0.12, ff.kS, EPSILON);
        assertEquals(2.5, ff.kV, EPSILON);
        assertEquals(0.1, ff.kA, EPSILON);
        assertEquals(0.35, ff.kG, EPSILON);
    }

    @Test
    @DisplayName("arm: creates gains with gravity")
    void arm_createsGainsWithGravity() {
        FeedforwardGains ff = FeedforwardGains.arm(0.15, 1.8, 0.05, 0.5);
        assertEquals(0.15, ff.kS, EPSILON);
        assertEquals(1.8, ff.kV, EPSILON);
        assertEquals(0.05, ff.kA, EPSILON);
        assertEquals(0.5, ff.kG, EPSILON);
    }

    // ===== ELEVATOR CALCULATIONS =====

    @Test
    @DisplayName("calculateElevator: no-arg returns kG (static hold)")
    void calculateElevator_noArg_returnsKG() {
        FeedforwardGains ff = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        assertEquals(0.35, ff.calculateElevator(), EPSILON);
    }

    @Test
    @DisplayName("calculateElevator: with velocity")
    void calculateElevator_withVelocity() {
        FeedforwardGains ff = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        // kS * sign(v) + kG + kV * v
        double expected = 0.12 * 1.0 + 0.35 + 2.5 * 1.0;
        assertEquals(expected, ff.calculateElevator(1.0), EPSILON);
    }

    @Test
    @DisplayName("calculateElevator: with velocity and acceleration")
    void calculateElevator_withAccel() {
        FeedforwardGains ff = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        // kS * sign(v) + kG + kV * v + kA * a
        double expected = 0.12 * 1.0 + 0.35 + 2.5 * 2.0 + 0.1 * 3.0;
        assertEquals(expected, ff.calculateElevator(2.0, 3.0), EPSILON);
    }

    @Test
    @DisplayName("calculateElevator: zero velocity contributes kG only (no kS)")
    void calculateElevator_zeroVelocity_onlyKG() {
        FeedforwardGains ff = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        assertEquals(0.35, ff.calculateElevator(0.0), EPSILON);
    }

    // ===== ARM CALCULATIONS =====

    @Test
    @DisplayName("calculateArm: at horizontal (0 rad) returns kG")
    void calculateArm_horizontal_returnsKG() {
        FeedforwardGains ff = FeedforwardGains.arm(0.15, 1.8, 0.05, 0.5);
        // cos(0) = 1, so kG * cos(0) = 0.5
        assertEquals(0.5, ff.calculateArm(0.0), EPSILON);
    }

    @Test
    @DisplayName("calculateArm: at vertical (pi/2) returns ~0 gravity")
    void calculateArm_vertical_nearZeroGravity() {
        FeedforwardGains ff = FeedforwardGains.arm(0.15, 1.8, 0.05, 0.5);
        // cos(pi/2) ≈ 0
        assertEquals(0.0, ff.calculateArm(Math.PI / 2), 0.001);
    }

    @Test
    @DisplayName("calculateArm: at 45 degrees, gravity is kG * cos(45)")
    void calculateArm_at45deg_cosineGravity() {
        FeedforwardGains ff = FeedforwardGains.arm(0.0, 0.0, 0.0, 1.0);
        double expected = Math.cos(Math.PI / 4); // ~0.7071
        assertEquals(expected, ff.calculateArm(Math.PI / 4), EPSILON);
    }

    @Test
    @DisplayName("calculateArm: with velocity includes kS and kV")
    void calculateArm_withVelocity() {
        FeedforwardGains ff = FeedforwardGains.arm(0.1, 2.0, 0.05, 0.5);
        // kS * sign(v) + kG * cos(angle) + kV * v
        double angle = Math.PI / 6; // 30 degrees
        double velocity = 1.5;
        double expected = 0.1 * 1.0 + 0.5 * Math.cos(angle) + 2.0 * velocity;
        assertEquals(expected, ff.calculateArm(angle, velocity), EPSILON);
    }

    // ===== SIMPLE CALCULATIONS =====

    @Test
    @DisplayName("calculateSimple: velocity only")
    void calculateSimple_velocity() {
        FeedforwardGains ff = FeedforwardGains.simple(0.1, 0.5);
        // kS * sign(v) + kV * v = 0.1 + 0.5 * 2 = 1.1
        assertEquals(1.1, ff.calculateSimple(2.0), EPSILON);
    }

    @Test
    @DisplayName("calculateSimple: negative velocity")
    void calculateSimple_negativeVelocity() {
        FeedforwardGains ff = FeedforwardGains.simple(0.1, 0.5);
        // kS * sign(-2) + kV * (-2) = -0.1 + -1.0 = -1.1
        assertEquals(-1.1, ff.calculateSimple(-2.0), EPSILON);
    }

    @Test
    @DisplayName("calculateSimple: with acceleration")
    void calculateSimple_withAccel() {
        FeedforwardGains ff = FeedforwardGains.simple(0.1, 0.5, 0.02);
        // kS * sign(v) + kV * v + kA * a = 0.1 + 0.5 * 1 + 0.02 * 3 = 0.66
        assertEquals(0.66, ff.calculateSimple(1.0, 3.0), EPSILON);
    }
}
