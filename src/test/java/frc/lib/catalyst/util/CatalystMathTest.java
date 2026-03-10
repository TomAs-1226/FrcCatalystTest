package frc.lib.catalyst.util;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for CatalystMath utility class.
 * Tests joystick processing, angle math, geometry helpers,
 * and physics calculations.
 */
class CatalystMathTest {

    private static final double EPSILON = 1e-6;

    // ===== DEADBAND TESTS =====

    @Test
    @DisplayName("deadband: value inside deadband returns 0")
    void deadband_insideDeadband_returnsZero() {
        assertEquals(0.0, CatalystMath.deadband(0.03, 0.05), EPSILON);
        assertEquals(0.0, CatalystMath.deadband(-0.03, 0.05), EPSILON);
        assertEquals(0.0, CatalystMath.deadband(0.0, 0.1), EPSILON);
    }

    @Test
    @DisplayName("deadband: value outside deadband is rescaled")
    void deadband_outsideDeadband_isRescaled() {
        double result = CatalystMath.deadband(0.5, 0.1);
        assertTrue(result > 0, "Result should be positive");
        assertTrue(result < 0.5, "Result should be less than input (rescaled from 0.1)");
    }

    @Test
    @DisplayName("deadband: value at 1.0 returns 1.0")
    void deadband_atMax_returnsOne() {
        assertEquals(1.0, CatalystMath.deadband(1.0, 0.1), EPSILON);
    }

    @Test
    @DisplayName("deadband: negative values are handled symmetrically")
    void deadband_negative_symmetrical() {
        double positive = CatalystMath.deadband(0.5, 0.1);
        double negative = CatalystMath.deadband(-0.5, 0.1);
        assertEquals(positive, -negative, EPSILON);
    }

    // ===== SQUARE INPUT TESTS =====

    @Test
    @DisplayName("squareInput: squares value while preserving sign")
    void squareInput_preservesSign() {
        assertEquals(0.25, CatalystMath.squareInput(0.5), EPSILON);
        assertEquals(-0.25, CatalystMath.squareInput(-0.5), EPSILON);
    }

    @Test
    @DisplayName("squareInput: zero returns zero")
    void squareInput_zero_returnsZero() {
        assertEquals(0.0, CatalystMath.squareInput(0.0), EPSILON);
    }

    @Test
    @DisplayName("squareInput: one returns one")
    void squareInput_one_returnsOne() {
        assertEquals(1.0, CatalystMath.squareInput(1.0), EPSILON);
        assertEquals(-1.0, CatalystMath.squareInput(-1.0), EPSILON);
    }

    // ===== ANGLE DIFFERENCE TESTS =====

    @Test
    @DisplayName("angleDifference: simple difference")
    void angleDifference_simple() {
        assertEquals(20.0, CatalystMath.angleDifference(350, 10), EPSILON);
    }

    @Test
    @DisplayName("angleDifference: wrapping around 360")
    void angleDifference_wrapping() {
        double diff = CatalystMath.angleDifference(350, 10);
        assertEquals(20.0, diff, EPSILON);
    }

    @Test
    @DisplayName("angleDifference: same angle is zero")
    void angleDifference_sameAngle_zero() {
        assertEquals(0.0, CatalystMath.angleDifference(90, 90), EPSILON);
    }

    @Test
    @DisplayName("angleDifference: opposite angles are 180")
    void angleDifference_opposite_180() {
        assertEquals(180.0, CatalystMath.angleDifference(0, 180), EPSILON);
    }

    // ===== NORMALIZE ANGLE TESTS =====

    @ParameterizedTest
    @CsvSource({
        "370,   10",
        "0,     0",
        "360,   0",
        "-10,   -10",
        "720,   0",
        "180,   180",
        "-180,  -180",
        "90,    90"
    })
    @DisplayName("normalizeAngle: maps angles to [-180, 180]")
    void normalizeAngle_correctRange(double input, double expected) {
        assertEquals(expected, CatalystMath.normalizeAngle(input), EPSILON);
    }

    // ===== CUBE INPUT TESTS =====

    @Test
    @DisplayName("cubeInput: cubes value while preserving sign")
    void cubeInput_preservesSign() {
        assertEquals(0.125, CatalystMath.cubeInput(0.5), EPSILON);
        assertEquals(-0.125, CatalystMath.cubeInput(-0.5), EPSILON);
    }

    // ===== PHYSICS HELPERS =====

    @Test
    @DisplayName("elevatorGravityFF: returns positive voltage")
    void elevatorGravityFF_positive() {
        double ff = CatalystMath.elevatorGravityFF(5.0, 0.0254, 10.0, 1);
        assertTrue(ff > 0, "Gravity FF should be positive, got: " + ff);
    }

    @Test
    @DisplayName("armGravityFF: zero at vertical")
    void armGravityFF_zeroAtVertical() {
        double ff = CatalystMath.armGravityFF(3.0, 0.5, 90.0, 50.0, 1);
        assertEquals(0.0, ff, 0.01, "Arm gravity FF should be ~0 at 90 degrees");
    }

    @Test
    @DisplayName("armGravityFF: maximum at horizontal")
    void armGravityFF_maxAtHorizontal() {
        double ffH = CatalystMath.armGravityFF(3.0, 0.5, 0.0, 50.0, 1);
        double ff45 = CatalystMath.armGravityFF(3.0, 0.5, 45.0, 50.0, 1);
        assertTrue(ffH > ff45, "FF should be larger at horizontal than 45 degrees");
    }
}
