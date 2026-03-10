package frc.robot.examples;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.util.InterpolatingTable;
import frc.lib.catalyst.util.MechanismVisualizer;

/**
 * BasicRobot - A simple FrcCatalyst example with an elevator and intake.
 *
 * <p>This class demonstrates the minimum code needed to build a functional
 * competition robot using FrcCatalyst. Copy this as a starting point for
 * your own robot project.
 *
 * <h2>What's included:</h2>
 * <ul>
 *   <li>Elevator (LinearMechanism) with 4 named presets</li>
 *   <li>Intake (RollerMechanism) with game piece detection</li>
 *   <li>Dashboard visualization (MechanismVisualizer)</li>
 *   <li>Simple autonomous routine</li>
 * </ul>
 *
 * <h2>Controls (Operator Controller - Port 1):</h2>
 * <ul>
 *   <li>A = Stow | B = Intake | X = Mid | Y = High</li>
 *   <li>Right Trigger = Intake | Left Trigger = Eject</li>
 *   <li>Left Bumper = Manual elevator jog</li>
 *   <li>Start = Zero elevator</li>
 * </ul>
 */
public class BasicRobot {

    // --- Controllers ---

    private final CommandXboxController operator = new CommandXboxController(1);

    // --- Mechanisms ---

    /**
     * Elevator - 2-stage cascade with Kraken X60 motors.
     *
     * Physical specs:
     * - 10:1 gear ratio
     * - 1-inch (0.0254m) spool radius
     * - 2-stage cascade (travel multiplied by 2)
     * - 5 kg carriage mass
     * - 0 to 1.2 meters travel range
     *
     * Safety features:
     * - 40A supply current limit
     * - 70C temperature cutoff
     * - Reverse limit switch on DIO 0 with auto-zero
     */
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
            .reverseLimitSwitch(0, true)
            .position("STOW", 0.0)
            .position("INTAKE", 0.15)
            .position("MID", 0.6)
            .position("HIGH", 1.1)
            .build()
    );

    /**
     * Intake - Roller mechanism with game piece detection.
     *
     * Features:
     * - 80% intake speed, 60% eject speed
     * - Stall detection: 25A sustained for 0.2s = game piece acquired
     * - Beam break sensor on DIO 1 for reliable detection
     * - Coast mode so game pieces aren't ejected when motor stops
     */
    private final RollerMechanism intake = new RollerMechanism(
        RollerMechanism.Config.builder()
            .name("Intake")
            .motor(3)
            .brakeMode(false)
            .intakeSpeed(0.8)
            .ejectSpeed(-0.6)
            .stallDetection(25, 0.2)
            .beamBreak(1)
            .build()
    );

    // --- Dashboard Visualization ---

    private final MechanismVisualizer viz =
        new MechanismVisualizer("BasicRobot", 1.0, 2.0);

    // --- Constructor ---

    public BasicRobot() {
        // Set up dashboard visualization
        var elevatorViz = viz.addElevator(
            "Elevator", 0.5, 0.0, 1.2, Color.kDodgerBlue
        );

        // Default commands
        elevator.setDefaultCommand(elevator.holdPosition());

        // -- Elevator presets --
        operator.a().onTrue(elevator.goTo("STOW"));
        operator.b().onTrue(elevator.goTo("INTAKE"));
        operator.x().onTrue(elevator.goTo("MID"));
        operator.y().onTrue(elevator.goTo("HIGH"));

        // -- Manual elevator jog --
        operator.leftBumper().whileTrue(
            elevator.jog(() -> -operator.getLeftY() * 4.0)
        );

        // -- Zero elevator --
        operator.start().onTrue(elevator.zero());

        // -- Intake controls --
        operator.rightTrigger(0.3).whileTrue(intake.intake());
        operator.leftTrigger(0.3).whileTrue(intake.eject());

        // -- Print estimated gains for tuning --
        // Build a config just for estimation (doesn't create hardware)
        var estimationConfig = LinearMechanism.Config.builder()
            .motorType(MotorType.KRAKEN_X60)
            .gearRatio(10.0)
            .drumRadius(0.0254)
            .stages(2)
            .mass(5.0)
            .build();
        System.out.println("=== Elevator Physics Estimates ===");
        System.out.println("Estimated kG: " + estimationConfig.estimateGravityFF() + " V");
        System.out.println("Max speed:    " + estimationConfig.estimateMaxSpeed() + " m/s");
    }

    // --- Autonomous ---

    /**
     * Simple autonomous: score a preloaded game piece, then stow.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Raise elevator to scoring position
            elevator.goToAndWait("HIGH", 0.05),

            // Eject game piece
            intake.eject(),
            Commands.waitSeconds(0.4),

            // Stow everything
            Commands.parallel(
                elevator.goTo("STOW"),
                intake.stopCommand()
            )
        );
    }
}
