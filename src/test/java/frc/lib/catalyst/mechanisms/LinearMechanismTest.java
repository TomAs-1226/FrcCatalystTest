package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.catalyst.hardware.MotorType;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for LinearMechanism (elevator).
 * Uses WPILib HAL simulation to test motor control, position tracking,
 * and command factories.
 */
class LinearMechanismTest {

    private LinearMechanism elevator;

    @BeforeEach
    void setUp() {
        // Initialize HAL for simulation
        assert HAL.initialize(500, 0);

        elevator = new LinearMechanism(
                LinearMechanism.Config.builder()
                        .name("TestElevator")
                        .motor(20) // Use unique CAN IDs to avoid conflicts
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(10.0)
                        .drumRadius(0.0254)
                        .range(0.0, 1.2)
                        .mass(5.0)
                        .pid(50, 0, 0.5)
                        .gravityGain(0.35)
                        .motionMagic(2.0, 4.0, 20.0)
                        .currentLimit(40)
                        .position("STOW", 0.0)
                        .position("MID", 0.6)
                        .position("HIGH", 1.1)
                        .positionTolerance(0.02)
                        .build()
        );
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    // ===== CONSTRUCTION TESTS =====

    @Test
    @DisplayName("constructor: elevator is created successfully")
    void constructor_createsSuccessfully() {
        assertNotNull(elevator, "Elevator should be created");
    }

    @Test
    @DisplayName("getPosition: starts at zero")
    void getPosition_startsAtZero() {
        assertEquals(0.0, elevator.getPosition(), 0.01);
    }

    @Test
    @DisplayName("getVelocity: starts at zero")
    void getVelocity_startsAtZero() {
        assertEquals(0.0, elevator.getVelocity(), 0.01);
    }

    @Test
    @DisplayName("getMotor: returns non-null motor")
    void getMotor_notNull() {
        assertNotNull(elevator.getMotor(), "Motor should not be null");
    }

    // ===== COMMAND FACTORY TESTS =====

    @Test
    @DisplayName("goTo: creates valid command for position name")
    void goTo_byName_createsCommand() {
        Command cmd = elevator.goTo("STOW");
        assertNotNull(cmd, "goTo command should not be null");
    }

    @Test
    @DisplayName("goTo: creates valid command for raw position")
    void goTo_byValue_createsCommand() {
        Command cmd = elevator.goTo(0.5);
        assertNotNull(cmd, "goTo command should not be null");
    }

    @Test
    @DisplayName("goToAndWait: creates valid command")
    void goToAndWait_createsCommand() {
        Command cmd = elevator.goToAndWait("HIGH", 0.02);
        assertNotNull(cmd, "goToAndWait command should not be null");
    }

    @Test
    @DisplayName("holdPosition: creates valid command")
    void holdPosition_createsCommand() {
        Command cmd = elevator.holdPosition();
        assertNotNull(cmd, "holdPosition command should not be null");
    }

    @Test
    @DisplayName("jog: creates valid command")
    void jog_createsCommand() {
        Command cmd = elevator.jog(() -> 0.5);
        assertNotNull(cmd, "jog command should not be null");
    }

    @Test
    @DisplayName("jogUp: creates valid command")
    void jogUp_createsCommand() {
        Command cmd = elevator.jogUp(0.3);
        assertNotNull(cmd, "jogUp command should not be null");
    }

    @Test
    @DisplayName("jogDown: creates valid command")
    void jogDown_createsCommand() {
        Command cmd = elevator.jogDown(0.3);
        assertNotNull(cmd, "jogDown command should not be null");
    }

    @Test
    @DisplayName("zero: creates valid command")
    void zero_createsCommand() {
        Command cmd = elevator.zero();
        assertNotNull(cmd, "zero command should not be null");
    }

    @Test
    @DisplayName("stopCommand: creates valid command")
    void stopCommand_createsCommand() {
        Command cmd = elevator.stopCommand();
        assertNotNull(cmd, "stopCommand should not be null");
    }

    // ===== NAMED POSITIONS =====

    @Test
    @DisplayName("goTo: all named positions are valid")
    void goTo_allNamedPositions() {
        assertDoesNotThrow(() -> elevator.goTo("STOW"));
        assertDoesNotThrow(() -> elevator.goTo("MID"));
        assertDoesNotThrow(() -> elevator.goTo("HIGH"));
    }

    @Test
    @DisplayName("goTo: invalid position throws")
    void goTo_invalidPosition_throws() {
        assertThrows(Exception.class, () -> elevator.goTo("NONEXISTENT"));
    }

    // ===== TRIGGER TESTS =====

    @Test
    @DisplayName("atPositionTrigger: creates trigger for position name")
    void atPositionTrigger_byName() {
        assertNotNull(elevator.atPositionTrigger("STOW"));
    }

    @Test
    @DisplayName("atPositionTrigger: creates trigger for value")
    void atPositionTrigger_byValue() {
        assertNotNull(elevator.atPositionTrigger(0.5));
    }

    @Test
    @DisplayName("forwardLimitTrigger: creates trigger")
    void forwardLimitTrigger_exists() {
        assertNotNull(elevator.forwardLimitTrigger());
    }

    @Test
    @DisplayName("reverseLimitTrigger: creates trigger")
    void reverseLimitTrigger_exists() {
        assertNotNull(elevator.reverseLimitTrigger());
    }

    // ===== CONFIG BUILDER VALIDATION =====

    @Test
    @DisplayName("config: follower motor works")
    void config_followerMotor() {
        assertDoesNotThrow(() -> new LinearMechanism(
                LinearMechanism.Config.builder()
                        .name("FollowerTest")
                        .motor(21)
                        .follower(22, true)
                        .motorType(MotorType.FALCON_500)
                        .gearRatio(5.0)
                        .drumRadius(0.02)
                        .range(0, 1)
                        .mass(2.0)
                        .pid(10, 0, 0)
                        .motionMagic(1, 2, 10)
                        .build()
        ));
    }

    @Test
    @DisplayName("config: multi-stage elevator works")
    void config_multiStage() {
        assertDoesNotThrow(() -> new LinearMechanism(
                LinearMechanism.Config.builder()
                        .name("MultiStageTest")
                        .motor(23)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(10.0)
                        .drumRadius(0.0254)
                        .stages(3)
                        .range(0, 2.0)
                        .mass(8.0)
                        .pid(50, 0, 1)
                        .motionMagic(1, 2, 10)
                        .build()
        ));
    }
}
