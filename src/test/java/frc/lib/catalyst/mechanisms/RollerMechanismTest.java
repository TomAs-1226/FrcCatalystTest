package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for RollerMechanism (intake/conveyor).
 */
class RollerMechanismTest {

    private RollerMechanism intake;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        intake = new RollerMechanism(
                RollerMechanism.Config.builder()
                        .name("TestIntake")
                        .motor(50)
                        .inverted(false)
                        .brakeMode(false)
                        .intakeSpeed(0.8)
                        .ejectSpeed(-0.6)
                        .stallDetection(25, 0.2)
                        .currentLimit(30)
                        .build()
        );
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    // ===== CONSTRUCTION =====

    @Test
    @DisplayName("constructor: intake created successfully")
    void constructor_success() {
        assertNotNull(intake);
    }

    // ===== COMMAND FACTORIES =====

    @Test
    @DisplayName("intake: creates valid command")
    void intake_createsCommand() {
        Command cmd = intake.intake();
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("eject: creates valid command")
    void eject_createsCommand() {
        Command cmd = intake.eject();
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("intakeContinuous: creates valid command")
    void intakeContinuous_createsCommand() {
        Command cmd = intake.intakeContinuous();
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("runAtSpeed: creates valid command")
    void runAtSpeed_createsCommand() {
        Command cmd = intake.runAtSpeed(0.5);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("runAtVoltage: creates valid command")
    void runAtVoltage_createsCommand() {
        Command cmd = intake.runAtVoltage(6.0);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("stopCommand: inherited from base")
    void stopCommand_exists() {
        Command cmd = intake.stopCommand();
        assertNotNull(cmd);
    }

    // ===== GAME PIECE DETECTION =====

    @Test
    @DisplayName("hasPiece: initially false")
    void hasPiece_initiallyFalse() {
        assertFalse(intake.hasPiece());
    }

    @Test
    @DisplayName("hasPieceTrigger: creates trigger")
    void hasPieceTrigger_exists() {
        assertNotNull(intake.hasPieceTrigger());
    }

    // ===== TELEMETRY =====

    @Test
    @DisplayName("getSpeed: starts at zero")
    void getSpeed_startsAtZero() {
        assertEquals(0.0, intake.getSpeed(), 0.01);
    }

    @Test
    @DisplayName("getCurrent: returns non-negative")
    void getCurrent_nonNegative() {
        assertTrue(intake.getCurrent() >= 0.0);
    }

    // ===== MOTOR =====

    @Test
    @DisplayName("getMotor: not null")
    void getMotor_notNull() {
        assertNotNull(intake.getMotor());
    }

    // ===== CONFIG VARIANTS =====

    @Test
    @DisplayName("config: inverted intake works")
    void config_inverted() {
        assertDoesNotThrow(() -> new RollerMechanism(
                RollerMechanism.Config.builder()
                        .name("InvertedIntake")
                        .motor(51)
                        .inverted(true)
                        .brakeMode(true)
                        .intakeSpeed(0.6)
                        .ejectSpeed(-0.4)
                        .currentLimit(25)
                        .build()
        ));
    }
}
