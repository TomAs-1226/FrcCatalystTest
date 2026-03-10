package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.*;
import frc.lib.catalyst.util.*;

/**
 * Test RobotContainer that creates every FrcCatalyst mechanism type
 * and wires them up for simulation testing.
 *
 * This validates that:
 * 1. All mechanisms can be instantiated with builder pattern
 * 2. All command factories produce valid commands
 * 3. Simulation physics models run without errors
 * 4. Telemetry publishes correctly
 * 5. Named presets, limit switches, and safety features work
 */
public class RobotContainer {

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== MECHANISMS =====

    /** Elevator: 2-stage cascade, Kraken X60, Motion Magic + limit switches */
    private final LinearMechanism elevator = new LinearMechanism(
            LinearMechanism.Config.builder()
                    .name("Elevator")
                    .motor(1)
                    .follower(2, true)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(10.0)
                    .drumRadius(0.0254) // 1 inch spool
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

    /** Arm: single-jointed, cosine gravity, with hard stop */
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
                    .position("AMP", 90)
                    .position("SCORE", 100)
                    .build()
    );

    /** Wrist: small rotational mechanism */
    private final RotationalMechanism wrist = new RotationalMechanism(
            RotationalMechanism.Config.builder()
                    .name("Wrist")
                    .motor(5)
                    .motorType(MotorType.FALCON_500)
                    .gearRatio(25.0)
                    .length(0.15)
                    .mass(0.5)
                    .range(-90, 90)
                    .pid(40, 0, 0.5)
                    .gravityGain(0.15)
                    .useCosineGravity(true)
                    .motionMagic(300, 600, 3000)
                    .position("STOW", 0)
                    .position("INTAKE", -45)
                    .position("SCORE", 60)
                    .build()
    );

    /** Shooter: dual flywheel for differential spin */
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

    /** Intake: roller with stall detection */
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

    /** Climber: winch mechanism */
    private final WinchMechanism climber = new WinchMechanism(
            WinchMechanism.Config.builder()
                    .name("Climber")
                    .motor(9)
                    .gearRatio(25.0)
                    .spoolRadius(0.02)
                    .range(0.0, 0.6)
                    .extendSpeed(0.8)
                    .retractSpeed(-1.0)
                    .currentLimit(80)
                    .build()
    );

    /** Elevator with WPILib ProfiledPID (alternative control) */
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

    // ===== COORDINATOR =====

    private final SuperstructureCoordinator superstructure = new SuperstructureCoordinator();

    public RobotContainer() {
        // Populate shooter lookup table
        shooterTable.add(1.0, 3000);
        shooterTable.add(2.0, 3500);
        shooterTable.add(3.0, 4200);
        shooterTable.add(5.0, 5000);

        // Configure superstructure
        superstructure
                .withLinear("elevator", elevator)
                .withRotational("arm", arm)
                .withRotational("wrist", wrist);

        superstructure.defineState("STOW")
                .setLinear("elevator", 0.0)
                .setRotational("arm", 0.0)
                .setRotational("wrist", 0.0)
                .done();

        superstructure.defineState("INTAKE")
                .setLinear("elevator", 0.3)
                .setRotational("arm", 15.0)
                .setRotational("wrist", -45.0)
                .done();

        superstructure.defineState("SCORE_HIGH")
                .setLinear("elevator", 1.1)
                .setRotational("arm", 100.0)
                .setRotational("wrist", 60.0)
                .done();

        // Set default commands
        elevator.setDefaultCommand(elevator.holdPosition());
        arm.setDefaultCommand(arm.holdPosition());
        wrist.setDefaultCommand(wrist.holdPosition());
        profiledElevator.setDefaultCommand(profiledElevator.holdPositionProfiled());

        // Put mechanism info on dashboard
        publishDashboard();

        System.out.println("RobotContainer initialized with all mechanisms!");
        System.out.println("  Elevator travel: " + elevator.getPosition() + "m");
        System.out.println("  Arm angle: " + arm.getAngle() + " deg");
        System.out.println("  Shooter table at 2.5m: " + shooterTable.get(2.5) + " RPM");

        // Verify utility classes work
        testUtilities();
    }

    /** Bind teleop controls */
    public void configureTeleopCommands() {
        // Elevator presets
        operator.a().onTrue(elevator.goTo("STOW"));
        operator.b().onTrue(elevator.goTo("INTAKE"));
        operator.x().onTrue(elevator.goTo("MID"));
        operator.y().onTrue(elevator.goTo("HIGH"));

        // Arm presets
        operator.povUp().onTrue(arm.goTo("SCORE"));
        operator.povDown().onTrue(arm.goTo("STOW"));
        operator.povLeft().onTrue(arm.goTo("INTAKE"));

        // Intake
        operator.rightTrigger().whileTrue(intake.intake());
        operator.leftTrigger().whileTrue(intake.eject());

        // Shooter
        operator.rightBumper().whileTrue(shooter.spinUp(70));

        // Climber
        driver.povUp().whileTrue(climber.extend());
        driver.povDown().whileTrue(climber.retract());

        // Superstructure transitions
        driver.a().onTrue(superstructure.transitionTo("STOW"));
        driver.y().onTrue(superstructure.transitionTo("SCORE_HIGH"));

        // Profiled elevator test
        driver.x().onTrue(profiledElevator.goToProfiled("TOP"));
        driver.b().onTrue(profiledElevator.goToProfiled("BOTTOM"));

        // Manual jog
        operator.leftBumper().whileTrue(
                elevator.jog(() -> -operator.getLeftY() * 4.0)
        );
    }

    /** Set up auto commands */
    public void configureAutoCommands() {
        // Simple auto: raise elevator, shoot, stow
        Command simpleAuto = Commands.sequence(
                elevator.goToAndWait("HIGH", 0.02),
                arm.goToAndWait("SCORE", 2.0),
                shooter.spinUpAndWait(70),
                Commands.waitSeconds(1.0),
                shooter.stopCommand(),
                superstructure.transitionTo("STOW")
        );
        simpleAuto.schedule();
    }

    private void publishDashboard() {
        // Characterization helpers (uses CatalystMotor directly)
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
        var armViz = viz.addArm("Arm", elevatorViz, 0.5,
                edu.wpi.first.wpilibj.util.Color.kRed);

        // Shooter interpolation test
        SmartDashboard.putNumber("ShooterTable/1.0m", shooterTable.get(1.0));
        SmartDashboard.putNumber("ShooterTable/2.5m", shooterTable.get(2.5));
        SmartDashboard.putNumber("ShooterTable/4.0m", shooterTable.get(4.0));
    }

    private void testUtilities() {
        // Test CatalystMath
        double deadbanded = CatalystMath.deadband(0.03, 0.05);
        assert deadbanded == 0 : "Deadband failed for value inside deadband";

        double squared = CatalystMath.squareInput(0.5);
        assert Math.abs(squared - 0.25) < 0.001 : "Square input failed";

        double angleDiff = CatalystMath.angleDifference(350, 10);
        assert Math.abs(angleDiff - 20) < 0.001 : "Angle difference failed";

        double normalized = CatalystMath.normalizeAngle(370);
        assert Math.abs(normalized - 10) < 0.001 : "Normalize angle failed";

        // Test SlewRateLimiter
        double slewed = driveRateLimiter.calculate(1.0);
        assert slewed > 0 && slewed <= 1.0 : "Slew rate limiter failed";

        // Test MovingAverage
        for (int i = 0; i < 10; i++) currentFilter.calculate(5.0);
        assert Math.abs(currentFilter.get() - 5.0) < 0.001 : "Moving average failed";

        // Test TimedBoolean
        boolean stalled = stallDetector.update(false);
        assert !stalled : "TimedBoolean should be false initially";

        // Test FeedforwardGains
        FeedforwardGains elevFF = FeedforwardGains.elevator(0.12, 2.5, 0.1, 0.35);
        double holdV = elevFF.calculateElevator();
        assert Math.abs(holdV - 0.35) < 0.001 : "Elevator hold FF failed";

        FeedforwardGains armFF = FeedforwardGains.arm(0.15, 1.8, 0.05, 0.5);
        double armHoldH = armFF.calculateArm(0); // horizontal
        assert Math.abs(armHoldH - 0.5) < 0.001 : "Arm hold FF at horizontal failed";
        double armHoldV = armFF.calculateArm(Math.PI / 2); // vertical
        assert Math.abs(armHoldV) < 0.001 : "Arm hold FF at vertical failed";

        // Test MotorType
        double krakenFreeSpeed = MotorType.KRAKEN_X60.freeSpeedRPS();
        assert krakenFreeSpeed > 0 : "Kraken free speed should be positive";

        System.out.println("  All utility tests passed!");
    }
}
