package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.catalyst.hardware.MotorType;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for FlywheelMechanism (shooter).
 */
class FlywheelMechanismTest {

    private FlywheelMechanism shooter;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        shooter = new FlywheelMechanism(
                FlywheelMechanism.Config.builder()
                        .name("TestShooter")
                        .motor(40)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(1.5)
                        .moi(0.01)
                        .pid(0.3, 0, 0)
                        .feedforward(0.12, 0.11)
                        .velocityTolerance(3.0)
                        .currentLimit(60)
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
    @DisplayName("constructor: shooter created successfully")
    void constructor_success() {
        assertNotNull(shooter);
    }

    @Test
    @DisplayName("getVelocity: starts at zero")
    void getVelocity_startsAtZero() {
        assertEquals(0.0, shooter.getVelocity(), 0.5);
    }

    @Test
    @DisplayName("getSetpoint: starts at zero")
    void getSetpoint_startsAtZero() {
        assertEquals(0.0, shooter.getSetpoint(), 0.1);
    }

    // ===== COMMAND FACTORIES =====

    @Test
    @DisplayName("spinUp: creates valid command")
    void spinUp_createsCommand() {
        Command cmd = shooter.spinUp(70.0);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("spinUpAndWait: creates valid command")
    void spinUpAndWait_createsCommand() {
        Command cmd = shooter.spinUpAndWait(70.0);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("runVoltage: creates valid command")
    void runVoltage_createsCommand() {
        Command cmd = shooter.runVoltage(6.0);
        assertNotNull(cmd);
    }

    @Test
    @DisplayName("stopCommand: inherited from base")
    void stopCommand_exists() {
        Command cmd = shooter.stopCommand();
        assertNotNull(cmd);
    }

    // ===== TRIGGERS =====

    @Test
    @DisplayName("atSpeedTrigger: creates trigger")
    void atSpeedTrigger_exists() {
        assertNotNull(shooter.atSpeedTrigger());
    }

    @Test
    @DisplayName("atSpeed: boolean method exists")
    void atSpeed_exists() {
        assertFalse(shooter.atSpeed(), "Should not be at speed when stopped");
    }

    // ===== DUAL MOTOR CONFIG =====

    @Test
    @DisplayName("config: dual motor shooter works")
    void config_dualMotor() {
        assertDoesNotThrow(() -> new FlywheelMechanism(
                FlywheelMechanism.Config.builder()
                        .name("DualShooter")
                        .motor(41)
                        .secondMotor(42)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(1.0)
                        .moi(0.01)
                        .pid(0.3, 0, 0)
                        .feedforward(0.12, 0.11)
                        .velocityTolerance(3.0)
                        .build()
        ));
    }

    @Test
    @DisplayName("spinUp: dual velocity command exists")
    void spinUp_dualVelocity() {
        FlywheelMechanism dual = new FlywheelMechanism(
                FlywheelMechanism.Config.builder()
                        .name("DualShooter2")
                        .motor(43)
                        .secondMotor(44)
                        .motorType(MotorType.KRAKEN_X60)
                        .gearRatio(1.0)
                        .moi(0.01)
                        .pid(0.3, 0, 0)
                        .feedforward(0.12, 0.11)
                        .velocityTolerance(3.0)
                        .build()
        );
        // spinUp with two velocities (differential spin)
        Command cmd = dual.spinUp(70.0, 50.0);
        assertNotNull(cmd);
    }

    // ===== MOTORS =====

    @Test
    @DisplayName("getPrimaryMotor: not null")
    void getPrimaryMotor_notNull() {
        assertNotNull(shooter.getPrimaryMotor());
    }
}
