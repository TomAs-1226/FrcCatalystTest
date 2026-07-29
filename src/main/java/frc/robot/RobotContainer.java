package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.*;
import frc.lib.catalyst.util.*;

import frc.lib.catalyst.statemachine.Handle;
import frc.lib.catalyst.statemachine.Routing;
import frc.lib.catalyst.statemachine.goals.ClawGoal;
import frc.lib.catalyst.statemachine.goals.FlywheelGoal;
import frc.lib.catalyst.statemachine.goals.LinearGoal;
import frc.lib.catalyst.statemachine.goals.RollerGoal;
import frc.lib.catalyst.statemachine.goals.RotationalGoal;
import frc.lib.catalyst.statemachine.goals.ServoGoal;
import frc.lib.catalyst.statemachine.goals.WinchGoal;
import frc.lib.catalyst.statemachine.goals.WristGoal;
import frc.lib.catalyst.statemachine.mech.Mechanisms;
import frc.lib.catalyst.statemachine.robot.Superstructure;

/**
 * Test RobotContainer that creates every FrcCatalyst mechanism type and drives them
 * with the modern {@link Superstructure} state machine (Catalyst 1.2+).
 *
 * <p>Updated for Catalyst v1.3.2. This validates that:
 * <ol>
 *   <li>Every mechanism builds with the builder pattern, including the newer
 *       {@link ClawMechanism}, {@link DifferentialWristMechanism}, and {@link ServoMechanism}.</li>
 *   <li>All nine bound mechanisms drive as one whole-robot {@link Superstructure} instead of the
 *       deprecated {@code SuperstructureCoordinator}: a legal-transition graph, a staged edge, an
 *       entry guard, and a global interlock.</li>
 *   <li>{@code explain()} prints a readable dump of the machine on demand.</li>
 *   <li>Simulation physics, telemetry, and command factories all run without errors.</li>
 * </ol>
 */
public class RobotContainer {

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== MECHANISMS (all bound into the superstructure below) =====

    /** Elevator: 2-stage cascade, Kraken X60, Motion Magic. */
    private final LinearMechanism elevator = new LinearMechanism(
            LinearMechanism.Config.builder()
                    .name("Elevator")
                    .motor(1)
                    .follower(2, true)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(10.0)
                    .drumRadius(0.0254)
                    .stages(2)
                    .range(0.0, 1.2)
                    .mass(5.0)
                    .pid(50, 0, 0.5)
                    .gravityGain(0.35)
                    .motionMagic(2.0, 4.0, 20.0)
                    .currentLimit(40)
                    .maxTemperature(70)
                    .position("STOW", 0.0)
                    .position("INTAKE", 0.3)
                    .position("MID", 0.7)
                    .position("HIGH", 1.1)
                    .positionTolerance(0.02)
                    .build()
    );

    /** Arm: single-jointed, cosine gravity. */
    private final RotationalMechanism arm = new RotationalMechanism(
            RotationalMechanism.Config.builder()
                    .name("Arm")
                    .motor(3)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(50.0)
                    .length(0.5)
                    .mass(3.0)
                    .range(-10, 120)
                    .pid(80, 0, 1.0)
                    .gravityGain(0.4)
                    .useCosineGravity(true)
                    .motionMagic(200, 400, 2000)
                    .currentLimit(30)
                    .position("STOW", 0)
                    .position("INTAKE", 15)
                    .position("SCORE", 100)
                    .build()
    );

    /** Wrist: differential (pitch + roll), native CTRE differential control (new in 1.2). */
    private final DifferentialWristMechanism wrist = new DifferentialWristMechanism(
            DifferentialWristMechanism.Config.builder()
                    .name("Wrist")
                    .leftMotor(4)
                    .rightMotor(5)
                    .gearRatio(40.0)
                    .pitchRange(-40, 40)
                    .rollRange(-40, 40)
                    .pid(40, 0, 0.5)
                    .motionMagic(2.0, 6.0, 60.0)
                    .tolerance(2.0)
                    .build()
    );

    /** Shooter: dual flywheel for differential spin. */
    private final FlywheelMechanism shooter = new FlywheelMechanism(
            FlywheelMechanism.Config.builder()
                    .name("Shooter")
                    .motor(6)
                    .secondMotor(7)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(1.5)
                    .moi(0.01)
                    .pid(0.3, 0, 0)
                    .feedforward(0.12, 0.11)
                    .velocityTolerance(3.0)
                    .currentLimit(60)
                    .build()
    );

    /** Intake: roller with stall detection. */
    private final RollerMechanism intake = new RollerMechanism(
            RollerMechanism.Config.builder()
                    .name("Intake")
                    .motor(8)
                    .inverted(false)
                    .brakeMode(false)
                    .intakeSpeed(0.8)
                    .ejectSpeed(-0.6)
                    .stallDetection(25, 0.2)
                    .currentLimit(30)
                    .build()
    );

    /** Claw: motor-driven gripper with game-piece detection (new in 1.2). */
    private final ClawMechanism claw = new ClawMechanism(
            ClawMechanism.Config.builder()
                    .name("Claw")
                    .motor(11)
                    .currentLimit(30)
                    .closeVoltage(3.0)
                    .openVoltage(-3.0)
                    .holdVoltage(1.0)
                    .build()
    );

    /** Climber: winch mechanism. */
    private final WinchMechanism climber = new WinchMechanism(
            WinchMechanism.Config.builder()
                    .name("Climber")
                    .motor(9)
                    .gearRatio(25.0)
                    .spoolRadius(0.02)
                    .range(0.0, 0.6)
                    .loadMass(8.0)
                    .extendSpeed(0.8)
                    .retractSpeed(-1.0)
                    .currentLimit(80)
                    .build()
    );

    /** Hood: PWM servo with named positions (new in 1.3.0). */
    private final ServoMechanism hood = new ServoMechanism(
            ServoMechanism.Config.builder()
                    .name("Hood")
                    .channel(0)
                    .angleRange(15, 55)
                    .position("CLOSE", 15)
                    .position("MID", 35)
                    .position("FAR", 55)
                    .build()
    );

    /** Elevator with WPILib ProfiledPID (alternative control, NOT in the state machine). */
    private final LinearMechanism profiledElevator = new LinearMechanism(
            LinearMechanism.Config.builder()
                    .name("ProfiledElevator")
                    .motor(10)
                    .motorType(MotorType.FALCON_500)
                    .gearRatio(8.0)
                    .drumRadius(0.02)
                    .range(0.0, 0.8)
                    .mass(3.0)
                    .pid(30, 0, 0.3)
                    .gravityGain(0.25)
                    .motionMagic(1.5, 3.0, 15.0)
                    .useWPILibProfile(12.0, 0, 0.5, 1.5, 3.0)
                    .position("BOTTOM", 0.0)
                    .position("TOP", 0.75)
                    .build()
    );

    // ===== UTILITIES =====

    private final InterpolatingTable shooterTable = new InterpolatingTable();
    private final SlewRateLimiter driveRateLimiter = new SlewRateLimiter(3.0, 5.0);
    private final MovingAverage currentFilter = new MovingAverage(10);
    private final TimedBoolean stallDetector = new TimedBoolean(0.3);

    // ===== STATE MACHINE =====

    /** The whole-robot states this test robot can be in. */
    public enum State { STOW, INTAKE, CARRY, AIM, SCORE, CLIMB }

    /** True once the driver has "armed" the endgame; gates entry to CLIMB. */
    private boolean climbArmed = false;

    private final Superstructure<State> superstructure = buildSuperstructure();

    public RobotContainer() {
        // Populate shooter lookup table
        shooterTable.add(1.0, 3000);
        shooterTable.add(2.0, 3500);
        shooterTable.add(3.0, 4200);
        shooterTable.add(5.0, 5000);

        // Only mechanisms NOT owned by the superstructure get a manual default command.
        // The superstructure installs its own GoalRunner default on every bound mechanism,
        // so setting one here would make build() fail.
        profiledElevator.setDefaultCommand(profiledElevator.holdPositionProfiled());

        publishDashboard();

        System.out.println("RobotContainer initialized with the Superstructure state machine.");
        System.out.println("  Elevator travel: " + elevator.getPosition() + "m");
        System.out.println("  Arm angle: " + arm.getAngle() + " deg");
        System.out.println("  Shooter table at 2.5m: " + shooterTable.get(2.5) + " RPM");

        // explain() prints a plain-language dump of what was built and why it might be stuck.
        System.out.println(superstructure.explain());

        testUtilities();
    }

    /**
     * Builds the whole-robot state machine over all bound mechanisms.
     *
     * <p>Shows the modern API that replaces the deprecated {@code SuperstructureCoordinator}:
     * a typed transition graph, a staged edge (elevator up before the arm swings), an entry
     * guard (cannot SCORE without a game piece), and a global interlock (nothing but STOW/CLIMB
     * while the winch is extended).
     */
    private Superstructure<State> buildSuperstructure() {
        Superstructure.Builder<State> b =
                Superstructure.builder(State.class, "TestSuperstructure");

        Handle<LinearGoal> hElevator = b.bind("elevator", Mechanisms.linear("elevator", elevator));
        Handle<RotationalGoal> hArm = b.bind("arm", Mechanisms.rotational("arm", arm));
        Handle<WristGoal> hWrist = b.bind("wrist", Mechanisms.wrist("wrist", wrist));
        Handle<FlywheelGoal> hShooter = b.bind("shooter", Mechanisms.flywheel("shooter", shooter));
        Handle<RollerGoal> hIntake = b.bind("intake", Mechanisms.roller("intake", intake));
        Handle<ClawGoal> hClaw = b.bind("claw", Mechanisms.claw("claw", claw));
        Handle<WinchGoal> hClimber = b.bind("climber", Mechanisms.winch("climber", climber));
        Handle<ServoGoal> hHood = b.bind("hood", Mechanisms.servo("hood", hood));

        return b
                .logPrefix("Test")
                .alertSubsystem("Test")
                .initialState(State.STOW)
                .routing(Routing.SHORTEST_PATH)
                .defaultTimeout(4.0)
                .historyCapacity(50)

                // Safe posture inherited by every state unless it says otherwise.
                .defaults(s -> s
                        .set(hWrist, WristGoal.level())
                        .set(hShooter, FlywheelGoal.idle())
                        .set(hIntake, RollerGoal.idle())
                        .set(hClaw, ClawGoal.hold())
                        .set(hClimber, WinchGoal.stop())
                        .set(hHood, ServoGoal.preset("CLOSE")))

                .state(State.STOW, s -> s
                        .set(hElevator, LinearGoal.preset("STOW"))
                        .set(hArm, RotationalGoal.degrees(0)))

                .state(State.INTAKE, s -> s
                        .set(hElevator, LinearGoal.preset("INTAKE"))
                        .set(hArm, RotationalGoal.degrees(15))
                        .set(hWrist, WristGoal.of(-20, 0))
                        .set(hIntake, RollerGoal.intakeUntilPiece(3.0))
                        .set(hClaw, ClawGoal.open())
                        .timeout(6.0))

                .state(State.CARRY, s -> s
                        .set(hElevator, LinearGoal.preset("MID"))
                        .set(hArm, RotationalGoal.degrees(15))
                        .set(hClaw, ClawGoal.close()))

                .state(State.AIM, s -> s
                        .set(hElevator, LinearGoal.preset("HIGH"))
                        .set(hArm, RotationalGoal.degrees(100))
                        .set(hWrist, WristGoal.of(10, 0))
                        .set(hShooter, FlywheelGoal.rps(45))
                        .set(hHood, ServoGoal.preset("FAR"))
                        .set(hClaw, ClawGoal.close()))

                // Release: needs a real game piece, and everything must settle before the claw counts.
                .state(State.SCORE, s -> s
                        .set(hElevator, LinearGoal.preset("HIGH"))
                        .set(hArm, RotationalGoal.degrees(100))
                        .set(hShooter, FlywheelGoal.rps(45))
                        .set(hHood, ServoGoal.preset("FAR"))
                        .set(hClaw, ClawGoal.open())
                        .set(hIntake, RollerGoal.eject(0.4))
                        .settleFor(0.2)
                        .entryGuard(claw::hasPiece, "no piece"))

                .state(State.CLIMB, s -> s
                        .set(hElevator, LinearGoal.preset("STOW"))
                        .set(hArm, RotationalGoal.degrees(0))
                        .set(hClimber, WinchGoal.extend())
                        .entryGuard(() -> climbArmed, "not armed")
                        .timeout(8.0)
                        .recoverTo(State.STOW))

                // STOW is the hub; SCORE hangs off AIM alone, so you cannot score from stowed in one hop.
                .allowBoth(State.STOW, State.INTAKE)
                .allowBoth(State.STOW, State.CARRY)
                .allowBoth(State.STOW, State.AIM)
                .allowBoth(State.STOW, State.CLIMB)
                .allow(State.INTAKE, State.CARRY)
                .allowBoth(State.CARRY, State.AIM)
                .allow(State.AIM, State.SCORE)
                .allow(State.SCORE, State.CARRY, State.STOW)

                // Raise the elevator BEFORE the arm and wrist swing out.
                .edge(State.STOW, State.AIM, e -> e
                        .stage(hElevator)
                        .stage(hArm, hWrist)
                        .timeout(5.0))

                // Global: while the winch is extended, only STOW and CLIMB are legal.
                .interlock("winchStowed",
                        () -> !climber.isFullyExtended(),
                        st -> st != State.CLIMB && st != State.STOW)

                .build();
    }

    /** Bind teleop controls. Drivers ask for whole-robot STATES, not individual mechanisms. */
    public void configureTeleopCommands() {
        // Superstructure states
        operator.a().onTrue(superstructure.goTo(State.STOW));
        operator.b().onTrue(superstructure.goTo(State.INTAKE));
        operator.x().onTrue(superstructure.goTo(State.CARRY));
        operator.y().onTrue(superstructure.goTo(State.AIM));
        operator.rightBumper().onTrue(superstructure.goTo(State.SCORE));

        // Arm the endgame, then climb.
        driver.back().onTrue(Commands.runOnce(() -> climbArmed = !climbArmed));
        driver.start().onTrue(superstructure.goTo(State.CLIMB));

        // Print explain() to the console on demand — the "why is it stuck?" button.
        operator.leftBumper().onTrue(Commands.runOnce(() -> System.out.println(superstructure.explain())));

        // Profiled elevator is standalone (not in the state machine).
        driver.x().onTrue(profiledElevator.goToProfiled("TOP"));
        driver.b().onTrue(profiledElevator.goToProfiled("BOTTOM"));
    }

    /** Set up auto commands: drive the machine through a scoring cycle. */
    public void configureAutoCommands() {
        Command simpleAuto = Commands.sequence(
                superstructure.goTo(State.INTAKE),
                superstructure.goTo(State.AIM),
                superstructure.goTo(State.SCORE),
                Commands.waitSeconds(0.5),
                superstructure.goTo(State.STOW)
        );
        simpleAuto.schedule();
    }

    private void publishDashboard() {
        // Characterization helper (uses CatalystMotor directly)
        CharacterizationHelper elevatorChar = new CharacterizationHelper(
                "Elevator", elevator, elevator.getMotor()
        );
        SmartDashboard.putData("Char/Elevator QS Fwd", elevatorChar.quasistaticForward());
        SmartDashboard.putData("Char/Elevator QS Rev", elevatorChar.quasistaticReverse());
        SmartDashboard.putData("Char/Elevator Dyn Fwd", elevatorChar.dynamicForward());
        SmartDashboard.putData("Char/Elevator Dyn Rev", elevatorChar.dynamicReverse());

        // Mechanism visualizer (name, width, height)
        MechanismVisualizer viz = new MechanismVisualizer("TestRobot", 1.0, 1.5);
        var elevatorViz = viz.addElevator("Elevator", 0.5, 0.0, 1.2,
                edu.wpi.first.wpilibj.util.Color.kBlue);
        viz.addArm("Arm", elevatorViz, 0.5, edu.wpi.first.wpilibj.util.Color.kRed);

        // Shooter interpolation test
        SmartDashboard.putNumber("ShooterTable/1.0m", shooterTable.get(1.0));
        SmartDashboard.putNumber("ShooterTable/2.5m", shooterTable.get(2.5));
        SmartDashboard.putNumber("ShooterTable/4.0m", shooterTable.get(4.0));
    }

    private void testUtilities() {
        // CatalystMath
        assert CatalystMath.deadband(0.03, 0.05) == 0 : "Deadband failed";
        assert Math.abs(CatalystMath.squareInput(0.5) - 0.25) < 0.001 : "Square input failed";
        assert Math.abs(CatalystMath.angleDifference(350, 10) - 20) < 0.001 : "Angle difference failed";
        assert Math.abs(CatalystMath.normalizeAngle(370) - 10) < 0.001 : "Normalize angle failed";

        // SlewRateLimiter
        double slewed = driveRateLimiter.calculate(1.0);
        assert slewed > 0 && slewed <= 1.0 : "Slew rate limiter failed";

        // MovingAverage
        for (int i = 0; i < 10; i++) currentFilter.calculate(5.0);
        assert Math.abs(currentFilter.get() - 5.0) < 0.001 : "Moving average failed";

        // TimedBoolean
        assert !stallDetector.update(false) : "TimedBoolean should be false initially";

        // FeedforwardGains
        FeedforwardGains elevFF = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        assert Math.abs(elevFF.calculateElevator() - 0.35) < 0.001 : "Elevator hold FF failed";

        // MotorType
        assert MotorType.KRAKEN_X60.freeSpeedRPS() > 0 : "Kraken free speed should be positive";

        System.out.println("  All utility checks passed!");
    }
}
