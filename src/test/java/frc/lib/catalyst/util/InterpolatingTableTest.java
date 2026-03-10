package frc.lib.catalyst.util;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for InterpolatingTable.
 * Validates linear interpolation, edge cases, and method chaining.
 */
class InterpolatingTableTest {

    private static final double EPSILON = 1e-6;
    private InterpolatingTable table;

    @BeforeEach
    void setUp() {
        table = new InterpolatingTable();
        // Typical shooter lookup table: distance (m) -> RPM
        table.add(1.0, 3000)
             .add(2.0, 3500)
             .add(3.0, 4200)
             .add(5.0, 5000);
    }

    @Test
    @DisplayName("get: exact key returns exact value")
    void get_exactKey_returnsExactValue() {
        assertEquals(3000, table.get(1.0), EPSILON);
        assertEquals(3500, table.get(2.0), EPSILON);
        assertEquals(4200, table.get(3.0), EPSILON);
        assertEquals(5000, table.get(5.0), EPSILON);
    }

    @Test
    @DisplayName("get: midpoint interpolation")
    void get_midpoint_interpolates() {
        // Halfway between 1.0 (3000) and 2.0 (3500) = 3250
        assertEquals(3250, table.get(1.5), EPSILON);
    }

    @Test
    @DisplayName("get: interpolation between non-adjacent entries")
    void get_interpolation_betweenEntries() {
        // Between 2.0 (3500) and 3.0 (4200) at 2.5
        // Expected: 3500 + 0.5 * (4200 - 3500) = 3850
        assertEquals(3850, table.get(2.5), EPSILON);
    }

    @Test
    @DisplayName("get: below minimum key clamps to first value")
    void get_belowMin_clampsToFirst() {
        assertEquals(3000, table.get(0.0), EPSILON);
        assertEquals(3000, table.get(-1.0), EPSILON);
    }

    @Test
    @DisplayName("get: above maximum key clamps to last value")
    void get_aboveMax_clampsToLast() {
        assertEquals(5000, table.get(6.0), EPSILON);
        assertEquals(5000, table.get(100.0), EPSILON);
    }

    @Test
    @DisplayName("add: method chaining works")
    void add_chainingWorks() {
        InterpolatingTable t = new InterpolatingTable()
                .add(0, 0)
                .add(1, 100)
                .add(2, 200);
        assertEquals(3, t.size());
        assertEquals(50, t.get(0.5), EPSILON);
    }

    @Test
    @DisplayName("size: returns correct count")
    void size_correctCount() {
        assertEquals(4, table.size());
    }

    @Test
    @DisplayName("clear: removes all entries")
    void clear_emptiesTable() {
        table.clear();
        assertEquals(0, table.size());
    }

    @Test
    @DisplayName("get: quarter-way interpolation")
    void get_quarterWay_interpolation() {
        // Between 3.0 (4200) and 5.0 (5000):
        // At 3.5 = 4200 + (0.5/2.0) * (5000-4200) = 4200 + 200 = 4400
        assertEquals(4400, table.get(3.5), EPSILON);
    }

    @Test
    @DisplayName("single entry table: always returns that value")
    void singleEntry_alwaysReturnsSameValue() {
        InterpolatingTable single = new InterpolatingTable().add(5.0, 42.0);
        assertEquals(42.0, single.get(0.0), EPSILON);
        assertEquals(42.0, single.get(5.0), EPSILON);
        assertEquals(42.0, single.get(10.0), EPSILON);
    }
}
