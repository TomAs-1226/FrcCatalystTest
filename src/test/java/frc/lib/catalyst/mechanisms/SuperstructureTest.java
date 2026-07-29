package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.statemachine.Handle;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.mech.Mechanisms;
import frc.lib.catalyst.statemachine.robot.Superstructure;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Integration tests for the modern {@link Superstructure} state machine (Catalyst 1.2+),
 * which replaces the deprecated {@code SuperstructureCoordinator}. Builds a small two-state
 * machine over a real elevator and arm and checks the graph, transitions, and {@code explain()}.
 */
class SuperstructureTest {

    /** The two states this tiny machine can be in. */
    enum St { STOW, HIGH }

    private LinearMechanism elevator;
    private RotationalMechanism arm;
    private Superstructure<St> ss;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        elevator = new LinearMechanism(
                LinearMechanism.Config.builder()
                        .name("SsElevator")
                        .motor(55)
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
                        .name("SsArm")
                        .motor(56)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(50.0)
                        .length(0.5)
                        .mass(3.0)
                        .range(-10, 120)
                        .pid(80, 0, 1.0)
                        .motionMagic(200, 400, 2000)
                        .build()
        );

        Superstructure.Builder<St> b = Superstructure.builder(St.class, "TestSs");
        Handle<LinearGoal> hElevator = b.bind("elevator", Mechanisms.linear("elevator", elevator));
        Handle<RotationalGoal> hArm = b.bind("arm", Mechanisms.rotational("arm", arm));

        ss = b
                .initialState(St.STOW)
                .state(St.STOW, s -> s
                        .set(hElevator, LinearGoal.preset("STOW"))
                        .set(hArm, RotationalGoal.degrees(0)))
                .state(St.HIGH, s -> s
                        .set(hElevator, LinearGoal.preset("HIGH"))
                        .set(hArm, RotationalGoal.degrees(90)))
                .allowBoth(St.STOW, St.HIGH)
                .build();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    @DisplayName("build: superstructure is created")
    void build_createsSuccessfully() {
        assertNotNull(ss);
    }

    @Test
    @DisplayName("current: starts in the initial state")
    void current_isInitialState() {
        assertEquals(St.STOW, ss.current());
    }

    @Test
    @DisplayName("goTo: returns a valid command for every declared state")
    void goTo_returnsCommands() {
        assertNotNull(ss.goTo(St.HIGH), "goTo(HIGH)");
        assertNotNull(ss.goTo(St.STOW), "goTo(STOW)");
    }

    @Test
    @DisplayName("explain: returns a non-empty human-readable dump")
    void explain_isNonEmpty() {
        String dump = ss.explain();
        assertNotNull(dump);
        assertFalse(dump.isBlank(), "explain() should describe the machine");
    }

    @Test
    @DisplayName("history / target / isAt: safe to query at rest")
    void queries_doNotThrow() {
        assertNotNull(ss.history());
        assertDoesNotThrow(() -> ss.target());
        assertDoesNotThrow(() -> ss.isAt(St.STOW));
    }
}
