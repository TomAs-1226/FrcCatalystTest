package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for DifferentialWristMechanism (pitch + roll, new in 1.2).
 */
class DifferentialWristMechanismTest {

    private DifferentialWristMechanism wrist;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        wrist = new DifferentialWristMechanism(
                DifferentialWristMechanism.Config.builder()
                        .name("TestWrist")
                        .leftMotor(53)
                        .rightMotor(54)
                        .gearRatio(40.0)
                        .pitchRange(-40, 40)
                        .rollRange(-40, 40)
                        .pid(40, 0, 0.5)
                        .motionMagic(2.0, 6.0, 60.0)
                        .tolerance(2.0)
                        .build()
        );
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    @DisplayName("constructor: wrist is created successfully")
    void constructor_createsSuccessfully() {
        assertNotNull(wrist);
    }

    @Test
    @DisplayName("command factories: all non-null")
    void commandFactories_nonNull() {
        assertNotNull(wrist.goTo(10, 0), "goTo(pitch,roll)");
        assertNotNull(wrist.goToAndWait(10, 0), "goToAndWait(pitch,roll)");
        assertNotNull(wrist.holdPosition(), "holdPosition");
        assertNotNull(wrist.zero(), "zero");
    }

    @Test
    @DisplayName("getPitch / getRoll: start near zero")
    void pitchRoll_startNearZero() {
        assertEquals(0.0, wrist.getPitch(), 0.5);
        assertEquals(0.0, wrist.getRoll(), 0.5);
    }

    @Test
    @DisplayName("ranges: report the configured limits")
    void ranges_matchConfig() {
        assertEquals(-40, wrist.getMinPitch(), 0.001);
        assertEquals(40, wrist.getMaxPitch(), 0.001);
        assertEquals(-40, wrist.getMinRoll(), 0.001);
        assertEquals(40, wrist.getMaxRoll(), 0.001);
    }

    @Test
    @DisplayName("atSetpointTrigger: not null")
    void atSetpointTrigger_notNull() {
        assertNotNull(wrist.atSetpointTrigger());
    }

    @Test
    @DisplayName("getCurrent: does not throw")
    void getCurrent_doesNotThrow() {
        assertDoesNotThrow(() -> wrist.getCurrent());
    }
}
