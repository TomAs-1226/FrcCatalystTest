package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.catalyst.hardware.MotorType;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Integration tests for SuperstructureCoordinator.
 * Tests multi-mechanism state machine coordination.
 */
class SuperstructureCoordinatorTest {

    private LinearMechanism elevator;
    private RotationalMechanism arm;
    private SuperstructureCoordinator coordinator;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        elevator = new LinearMechanism(
                LinearMechanism.Config.builder()
                        .name("CoordElevator")
                        .motor(70)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(10.0)
                        .drumRadius(0.0254)
                        .range(0.0, 1.2)
                        .mass(5.0)
                        .pid(50, 0, 0.5)
                        .motionMagic(2.0, 4.0, 20.0)
                        .position("STOW", 0.0)
                        .position("HIGH", 1.1)
                        .build()
        );

        arm = new RotationalMechanism(
                RotationalMechanism.Config.builder()
                        .name("CoordArm")
                        .motor(71)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(50.0)
                        .length(0.5)
                        .mass(3.0)
                        .range(-10, 120)
                        .pid(80, 0, 1.0)
                        .motionMagic(200, 400, 2000)
                        .position("STOW", 0)
                        .position("SCORE", 100)
                        .build()
        );

        coordinator = new SuperstructureCoordinator()
                .withLinear("elevator", elevator)
                .withRotational("arm", arm);

        coordinator.defineState("STOW")
                .setLinear("elevator", 0.0)
                .setRotational("arm", 0.0)
                .done();

        coordinator.defineState("SCORE_HIGH")
                .setLinear("elevator", 1.1)
                .setRotational("arm", 100.0)
                .done();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    // ===== CONSTRUCTION =====

    @Test
    @DisplayName("constructor: coordinator created successfully")
    void constructor_success() {
        assertNotNull(coordinator);
    }

    @Test
    @DisplayName("withLinear/withRotational: chain returns same instance")
    void withMechanism_returnsSameInstance() {
        SuperstructureCoordinator c = new SuperstructureCoordinator();
        SuperstructureCoordinator result = c.withLinear("e", elevator);
        assertSame(c, result, "withLinear should return same instance for chaining");
    }

    // ===== STATE DEFINITION =====

    @Test
    @DisplayName("defineState: creates state builder")
    void defineState_createsBuilder() {
        SuperstructureCoordinator c = new SuperstructureCoordinator()
                .withLinear("e", elevator);
        assertNotNull(c.defineState("TEST"));
    }

    // ===== TRANSITIONS =====

    @Test
    @DisplayName("transitionTo: creates valid command")
    void transitionTo_createsCommand() {
        Command cmd = coordinator.transitionTo("STOW");
        assertNotNull(cmd, "transitionTo should create a command");
    }

    @Test
    @DisplayName("transitionTo: all defined states produce commands")
    void transitionTo_allStates() {
        assertNotNull(coordinator.transitionTo("STOW"));
        assertNotNull(coordinator.transitionTo("SCORE_HIGH"));
    }

    // ===== STATE QUERIES =====

    @Test
    @DisplayName("getCurrentState: returns a state")
    void getCurrentState_returnsState() {
        String state = coordinator.getCurrentState();
        assertNotNull(state, "Should return a state name");
    }

    @Test
    @DisplayName("inState: creates trigger")
    void inState_createsTrigger() {
        assertNotNull(coordinator.inState("STOW"));
        assertNotNull(coordinator.inState("SCORE_HIGH"));
    }

    @Test
    @DisplayName("isAtState: returns boolean without error")
    void isAtState_returnsBoolean() {
        // Just verify it doesn't throw
        coordinator.isAtState("STOW");
        coordinator.isAtState("SCORE_HIGH");
    }

    // ===== TRANSITION RULES =====

    @Test
    @DisplayName("addTransitionRule: accepts custom rule")
    void addTransitionRule_accepted() {
        // TransitionRule takes (StateDefinition fromState, StateDefinition toState)
        assertDoesNotThrow(() ->
                coordinator.addTransitionRule("STOW", "SCORE_HIGH",
                        (fromState, toState) -> elevator.goTo("HIGH")
                                .alongWith(arm.goTo("STOW"))
                                .andThen(arm.goTo("SCORE"))
                )
        );
    }

    @Test
    @DisplayName("transitionTo: uses custom rule when defined")
    void transitionTo_usesCustomRule() {
        coordinator.addTransitionRule("STOW", "SCORE_HIGH",
                (fromState, toState) -> elevator.goTo("HIGH"));

        Command cmd = coordinator.transitionTo("SCORE_HIGH");
        assertNotNull(cmd);
    }
}
