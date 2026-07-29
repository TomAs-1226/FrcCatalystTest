package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for ServoMechanism (PWM servo, new in 1.3.0).
 */
class ServoMechanismTest {

    private ServoMechanism hood;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        hood = new ServoMechanism(
                ServoMechanism.Config.builder()
                        .name("TestHood")
                        .channel(1)
                        .angleRange(15, 55)
                        .position("CLOSE", 15)
                        .position("MID", 35)
                        .position("FAR", 55)
                        .build()
        );
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        // Free the PWM channel so the next test method can re-allocate it in the same fork.
        hood.getServo().close();
    }

    @Test
    @DisplayName("constructor: servo is created successfully")
    void constructor_createsSuccessfully() {
        assertNotNull(hood);
    }

    @Test
    @DisplayName("command factories: all non-null")
    void commandFactories_nonNull() {
        assertNotNull(hood.goTo(30), "goTo(degrees)");
        assertNotNull(hood.goTo("FAR"), "goTo(name)");
        assertNotNull(hood.hold(), "hold");
    }

    @Test
    @DisplayName("angle range: reports configured limits")
    void angleRange_matchesConfig() {
        assertEquals(15, hood.getMinAngle(), 0.001);
        assertEquals(55, hood.getMaxAngle(), 0.001);
    }

    @Test
    @DisplayName("setAngle: clamps within the configured range")
    void setAngle_clampsToRange() {
        hood.setAngle(100); // above max
        assertTrue(hood.getAngle() <= 55.001, "angle should clamp to max");
        hood.setAngle(-100); // below min
        assertTrue(hood.getAngle() >= 14.999, "angle should clamp to min");
    }

    @Test
    @DisplayName("goTo invalid name: throws")
    void goTo_invalidName_throws() {
        assertThrows(Exception.class, () -> hood.goTo("NONEXISTENT"));
    }

    @Test
    @DisplayName("getPosition: does not throw")
    void getPosition_doesNotThrow() {
        assertDoesNotThrow(() -> hood.getPosition());
    }
}
