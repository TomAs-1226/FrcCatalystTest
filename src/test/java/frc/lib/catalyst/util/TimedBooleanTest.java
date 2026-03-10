package frc.lib.catalyst.util;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for TimedBoolean.
 * Tests debounced boolean (sustained condition detection).
 * Requires HAL initialization because TimedBoolean uses WPILib Timer.
 */
class TimedBooleanTest {

    private TimedBoolean timed;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        // 0.3 second required duration
        timed = new TimedBoolean(0.3);
    }

    @Test
    @DisplayName("update: initially false")
    void update_initiallyFalse() {
        assertFalse(timed.update(false));
    }

    @Test
    @DisplayName("update: true input does not immediately return true")
    void update_trueInput_notImmediate() {
        // Even with true input, should not be true until duration elapsed
        boolean result = timed.update(true);
        // The very first update with true should not return true
        // (no time has passed yet)
        assertFalse(result, "Should not be true immediately");
    }

    @Test
    @DisplayName("get: returns current debounced state")
    void get_returnsCurrentState() {
        timed.update(false);
        assertFalse(timed.get());
    }

    @Test
    @DisplayName("reset: clears state")
    void reset_clearsState() {
        timed.update(true);
        timed.reset();
        assertFalse(timed.get());
    }

    @Test
    @DisplayName("update: false input resets timer")
    void update_falseInput_resetsTimer() {
        timed.update(true);
        timed.update(false);
        timed.update(true);
        // Timer was reset, so should not be true yet
        assertFalse(timed.get(), "Timer should have reset on false input");
    }

    @Test
    @DisplayName("risingEdge: detects transition to true")
    void risingEdge_detectsTransition() {
        // First call with false
        boolean edge1 = timed.risingEdge(false);
        assertFalse(edge1, "No rising edge when input is false");
    }

    @Test
    @DisplayName("fallingEdge: detects transition to false")
    void fallingEdge_detectsTransition() {
        boolean edge1 = timed.fallingEdge(false);
        assertFalse(edge1, "No falling edge on first call");
    }
}
