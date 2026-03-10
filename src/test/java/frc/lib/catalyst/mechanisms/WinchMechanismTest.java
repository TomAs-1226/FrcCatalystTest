package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for WinchMechanism (climber).
 */
class WinchMechanismTest {

    private WinchMechanism climber;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        climber = new WinchMechanism(
                WinchMechanism.Config.builder()
                        .name("TestClimber")
                        .motor(60)
                        .gearRatio(25.0)
                        .spoolRadius(0.02)
                        .range(0.0, 0.6)
                        .extendSpeed(0.8)
                        .retractSpeed(-1.0)
                        .currentLimit(80)
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
    @DisplayName("constructor: climber created successfully")
    void constructor_success() {
        assertNotNull(climber);
    }

    // ===== COMMAND FACTORIES =====

    @Test
    @DisplayName("extend: creates valid command")
    void extend_createsCommand() {
        Command cmd = climber.extend();
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("retract: creates valid command")
    void retract_createsCommand() {
        Command cmd = climber.retract();
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("runAtSpeed: creates valid command")
    void runAtSpeed_createsCommand() {
        Command cmd = climber.runAtSpeed(0.5);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("manualControl: creates valid command")
    void manualControl_createsCommand() {
        Command cmd = climber.manualControl(() -> 0.5);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("zero: creates valid command")
    void zero_createsCommand() {
        Command cmd = climber.zero();
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("stopCommand: inherited from base")
    void stopCommand_exists() {
        Command cmd = climber.stopCommand();
        assertNotNull(cmd);
    }

    // ===== POSITION =====

    @Test
    @DisplayName("getPosition: returns non-NaN")
    void getPosition_notNaN() {
        assertFalse(Double.isNaN(climber.getPosition()));
    }

    @Test
    @DisplayName("getCurrent: returns non-negative")
    void getCurrent_nonNegative() {
        assertTrue(climber.getCurrent() >= 0.0);
    }

    // ===== TRIGGERS =====

    @Test
    @DisplayName("fullyExtendedTrigger: creates trigger")
    void fullyExtendedTrigger_exists() {
        assertNotNull(climber.fullyExtendedTrigger());
    }

    @Test
    @DisplayName("fullyRetractedTrigger: creates trigger")
    void fullyRetractedTrigger_exists() {
        assertNotNull(climber.fullyRetractedTrigger());
    }

    @Test
    @DisplayName("isFullyExtended: initially depends on start position")
    void isFullyExtended_queryable() {
        // Just verify it doesn't throw
        climber.isFullyExtended();
    }

    @Test
    @DisplayName("isFullyRetracted: initially depends on start position")
    void isFullyRetracted_queryable() {
        climber.isFullyRetracted();
    }

    // ===== MOTOR =====

    @Test
    @DisplayName("getMotor: not null")
    void getMotor_notNull() {
        assertNotNull(climber.getMotor());
    }
}
