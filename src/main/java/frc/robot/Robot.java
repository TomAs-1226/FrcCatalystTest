package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * FrcCatalyst Test Robot - exercises all mechanisms in simulation.
 * Run with: ./gradlew simulateJava
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        System.out.println("=== FrcCatalyst Test Robot Initialized ===");
        System.out.println("All mechanisms created successfully in simulation.");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        System.out.println("=== Teleop Enabled ===");
        robotContainer.configureTeleopCommands();
    }

    @Override
    public void autonomousInit() {
        System.out.println("=== Auto Enabled ===");
        robotContainer.configureAutoCommands();
    }

    @Override
    public void disabledInit() {
        System.out.println("=== Disabled ===");
    }

    @Override
    public void simulationPeriodic() {
        // WPILib calls simulationPeriodic on all subsystems automatically
    }
}
