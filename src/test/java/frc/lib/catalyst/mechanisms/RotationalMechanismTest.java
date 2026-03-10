package frc.lib.catalyst.mechanisms;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.catalyst.hardware.MotorType;

import org.junit.jupiter.api.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Simulation integration tests for RotationalMechanism (arm/wrist).
 */
class RotationalMechanismTest {

    private RotationalMechanism arm;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        arm = new RotationalMechanism(
                RotationalMechanism.Config.builder()
                        .name("TestArm")
                        .motor(30)
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
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    // ===== CONSTRUCTION =====

    @Test
    @DisplayName("constructor: arm is created successfully")
    void constructor_createsSuccessfully() {
        assertNotNull(arm);
    }

    @Test
    @DisplayName("getAngle: starts at expected position")
    void getAngle_startsAtExpected() {
        // Should start near 0 (or wherever sim initializes)
        double angle = arm.getAngle();
        // Just verify it returns a number without errors
        assertFalse(Double.isNaN(angle), "Angle should not be NaN");
    }

    @Test
    @DisplayName("getAngularVelocity: starts at zero")
    void getAngularVelocity_startsAtZero() {
        assertEquals(0.0, arm.getAngularVelocity(), 1.0);
    }

    @Test
    @DisplayName("getMotor: returns non-null motor")
    void getMotor_notNull() {
        assertNotNull(arm.getMotor());
    }

    // ===== COMMAND FACTORIES =====

    @Test
    @DisplayName("goTo: by name creates command")
    void goTo_byName() {
        assertNotNull(arm.goTo("STOW"));
        assertNotNull(arm.goTo("INTAKE"));
        assertNotNull(arm.goTo("AMP"));
        assertNotNull(arm.goTo("SCORE"));
    }

    @Test
    @DisplayName("goTo: by angle creates command")
    void goTo_byAngle() {
        assertNotNull(arm.goTo(45.0));
    }

    @Test
    @DisplayName("goToAndWait: creates command")
    void goToAndWait_createsCommand() {
        assertNotNull(arm.goToAndWait("SCORE", 2.0));
    }

    @Test
    @DisplayName("holdPosition: creates command")
    void holdPosition_createsCommand() {
        assertNotNull(arm.holdPosition());
    }

    @Test
    @DisplayName("jog: creates command")
    void jog_createsCommand() {
        assertNotNull(arm.jog(() -> 1.0));
    }

    @Test
    @DisplayName("jogCW: creates command")
    void jogCW_createsCommand() {
        assertNotNull(arm.jogCW(0.3));
    }

    @Test
    @DisplayName("jogCCW: creates command")
    void jogCCW_createsCommand() {
        assertNotNull(arm.jogCCW(0.3));
    }

    @Test
    @DisplayName("zero: creates command")
    void zero_createsCommand() {
        assertNotNull(arm.zero());
    }

    @Test
    @DisplayName("stopCommand: creates command")
    void stopCommand_createsCommand() {
        assertNotNull(arm.stopCommand());
    }

    // ===== NAMED POSITION VALIDATION =====

    @Test
    @DisplayName("goTo: invalid position throws")
    void goTo_invalidName_throws() {
        assertThrows(Exception.class, () -> arm.goTo("INVALID_POS"));
    }

    // ===== TRIGGERS =====

    @Test
    @DisplayName("atAngleTrigger: creates trigger")
    void atAngleTrigger_exists() {
        assertNotNull(arm.atAngleTrigger(90.0));
    }

    @Test
    @DisplayName("atPositionTrigger: creates trigger by name")
    void atPositionTrigger_byName() {
        assertNotNull(arm.atPositionTrigger("SCORE"));
    }

    // ===== COSINE GRAVITY CONFIG =====

    @Test
    @DisplayName("config: wrist without cosine gravity works")
    void config_noCosineGravity() {
        assertDoesNotThrow(() -> new RotationalMechanism(
                RotationalMechanism.Config.builder()
                        .name("WristNoCosine")
                        .motor(31)
                        .motorType(MotorType.FALCON_500)
                        .gearRatio(25.0)
                        .length(0.15)
                        .mass(0.5)
                        .range(-90, 90)
                        .pid(40, 0, 0.5)
                        .motionMagic(300, 600, 3000)
                        .build()
        ));
    }
}
