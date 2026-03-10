package frc.lib.catalyst.util;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for SlewRateLimiter.
 * Tests asymmetric rate limiting for acceleration vs braking profiles.
 * Requires HAL initialization because SlewRateLimiter uses WPILib Timer.
 */
class SlewRateLimiterTest {

    private static final double EPSILON = 1e-3;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @Test
    @DisplayName("constructor: symmetric rate limiter")
    void constructor_symmetric() {
        SlewRateLimiter limiter = new SlewRateLimiter(5.0);
        // First call should return a limited value
        double result = limiter.calculate(1.0);
        assertTrue(result >= 0.0 && result <= 1.0,
                "Result should be between 0 and 1, got: " + result);
    }

    @Test
    @DisplayName("constructor: asymmetric rate limiter")
    void constructor_asymmetric() {
        // Accel at 3.0, brake at 5.0
        SlewRateLimiter limiter = new SlewRateLimiter(3.0, 5.0);
        double result = limiter.calculate(1.0);
        assertTrue(result >= 0.0 && result <= 1.0,
                "First calculation should be limited, got: " + result);
    }

    @Test
    @DisplayName("calculate: rate-limits large step input")
    void calculate_limitsLargeStep() {
        SlewRateLimiter limiter = new SlewRateLimiter(5.0);
        // Step from 0 to 1 should be limited
        double first = limiter.calculate(1.0);
        assertTrue(first < 1.0, "Should not reach 1.0 in one step, got: " + first);
        assertTrue(first > 0.0, "Should have moved towards 1.0, got: " + first);
    }

    @Test
    @DisplayName("calculate: small step not limited")
    void calculate_smallStep_notLimited() {
        SlewRateLimiter limiter = new SlewRateLimiter(1000.0);
        // Very high rate limit should not limit a step to 0.5
        double result = limiter.calculate(0.5);
        // With a very high rate limit, even the first call should get close
        assertTrue(result > 0.3, "With high rate limit, should be close to target, got: " + result);
    }

    @Test
    @DisplayName("reset: sets output to specific value")
    void reset_setsValue() {
        SlewRateLimiter limiter = new SlewRateLimiter(5.0);
        limiter.calculate(1.0);
        limiter.reset(0.5);
        assertEquals(0.5, limiter.get(), EPSILON);
    }

    @Test
    @DisplayName("reset: no-arg resets to zero")
    void reset_noArg_resetsToZero() {
        SlewRateLimiter limiter = new SlewRateLimiter(5.0);
        limiter.calculate(1.0);
        limiter.reset();
        assertEquals(0.0, limiter.get(), EPSILON);
    }

    @Test
    @DisplayName("get: returns current output value")
    void get_returnsCurrentValue() {
        SlewRateLimiter limiter = new SlewRateLimiter(5.0);
        double calculated = limiter.calculate(0.5);
        assertEquals(calculated, limiter.get(), EPSILON);
    }
}
