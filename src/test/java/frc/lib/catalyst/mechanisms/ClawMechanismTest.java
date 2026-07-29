package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for ClawMechanism (motor-driven gripper, new in 1.2).
 */
class ClawMechanismTest {

    private ClawMechanism claw;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        claw = new ClawMechanism(
                ClawMechanism.Config.builder()
                        .name("TestClaw")
                        .motor(52)
                        .currentLimit(30)
                        .closeVoltage(3.0)
                        .openVoltage(-3.0)
                        .holdVoltage(1.0)
                        .build()
        );
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    @DisplayName("constructor: claw is created successfully")
    void constructor_createsSuccessfully() {
        assertNotNull(claw);
    }

    @Test
    @DisplayName("command factories: all non-null")
    void commandFactories_nonNull() {
        assertNotNull(claw.close(), "close");
        assertNotNull(claw.open(), "open");
        assertNotNull(claw.hold(), "hold");
        assertNotNull(claw.closeUntilGripped(), "closeUntilGripped");
        assertNotNull(claw.runAtVoltage(2.0), "runAtVoltage");
        assertNotNull(claw.resetPieceDetection(), "resetPieceDetection");
    }

    @Test
    @DisplayName("hasPiece: false before anything is grabbed")
    void hasPiece_falseInitially() {
        assertFalse(claw.hasPiece(), "no piece should be detected at construction");
    }

    @Test
    @DisplayName("setSimHasPiece: drives the detection flag in simulation")
    void setSimHasPiece_togglesDetection() {
        claw.setSimHasPiece(true);
        assertTrue(claw.hasPiece(), "sim piece should register as detected");
        claw.setSimHasPiece(false);
        assertFalse(claw.hasPiece(), "sim piece should clear");
    }

    @Test
    @DisplayName("hasPieceTrigger: not null")
    void hasPieceTrigger_notNull() {
        assertNotNull(claw.hasPieceTrigger());
    }

    @Test
    @DisplayName("getCurrent: does not throw")
    void getCurrent_doesNotThrow() {
        assertDoesNotThrow(() -> claw.getCurrent());
    }
}
