package frc.lib.catalyst.util;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for MovingAverage.
 * Tests sliding window average filter behavior.
 */
class MovingAverageTest {

    private static final double EPSILON = 1e-6;
    private MovingAverage filter;

    @BeforeEach
    void setUp() {
        filter = new MovingAverage(5);
    }

    @Test
    @DisplayName("calculate: returns running average")
    void calculate_returnsAverage() {
        filter.calculate(10.0);
        filter.calculate(20.0);
        // Average of [10, 20] = 15
        assertEquals(15.0, filter.get(), EPSILON);
    }

    @Test
    @DisplayName("calculate: constant input returns that value")
    void calculate_constant_returnsSameValue() {
        for (int i = 0; i < 10; i++) {
            filter.calculate(5.0);
        }
        assertEquals(5.0, filter.get(), EPSILON);
    }

    @Test
    @DisplayName("calculate: window slides correctly")
    void calculate_windowSlides() {
        // Fill window with 10s (window size = 5)
        for (int i = 0; i < 5; i++) filter.calculate(10.0);
        assertEquals(10.0, filter.get(), EPSILON);

        // Now push 20s - old 10s should slide out
        for (int i = 0; i < 5; i++) filter.calculate(20.0);
        assertEquals(20.0, filter.get(), EPSILON);
    }

    @Test
    @DisplayName("isFull: returns false before window filled")
    void isFull_beforeFilled_false() {
        filter.calculate(1.0);
        assertFalse(filter.isFull());
    }

    @Test
    @DisplayName("isFull: returns true after window filled")
    void isFull_afterFilled_true() {
        for (int i = 0; i < 5; i++) filter.calculate(1.0);
        assertTrue(filter.isFull());
    }

    @Test
    @DisplayName("reset: clears all samples")
    void reset_clearsAll() {
        for (int i = 0; i < 5; i++) filter.calculate(100.0);
        filter.reset();
        assertFalse(filter.isFull());
        assertEquals(0, filter.getCount());
    }

    @Test
    @DisplayName("getCount: tracks number of samples")
    void getCount_tracksCount() {
        assertEquals(0, filter.getCount());
        filter.calculate(1.0);
        assertEquals(1, filter.getCount());
        filter.calculate(2.0);
        assertEquals(2, filter.getCount());
    }

    @Test
    @DisplayName("getCount: does not exceed window size")
    void getCount_maxAtWindowSize() {
        for (int i = 0; i < 20; i++) filter.calculate(1.0);
        assertEquals(5, filter.getCount());
    }

    @Test
    @DisplayName("single sample: average equals that sample")
    void singleSample_equalsItself() {
        filter.calculate(42.0);
        assertEquals(42.0, filter.get(), EPSILON);
    }

    @Test
    @DisplayName("ascending values: average is correct")
    void ascendingValues_correctAverage() {
        // Window size 5, add 1,2,3,4,5
        filter.calculate(1);
        filter.calculate(2);
        filter.calculate(3);
        filter.calculate(4);
        filter.calculate(5);
        // Average = (1+2+3+4+5)/5 = 3.0
        assertEquals(3.0, filter.get(), EPSILON);
    }
}
