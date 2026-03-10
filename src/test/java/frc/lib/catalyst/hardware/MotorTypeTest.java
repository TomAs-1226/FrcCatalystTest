package frc.lib.catalyst.hardware;

import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for MotorType enum.
 * Validates motor constants and physics calculations.
 */
class MotorTypeTest {

    private static final double EPSILON = 1e-3;

    // ===== BASIC CONSTANTS =====

    @Test
    @DisplayName("KRAKEN_X60: free speed is positive")
    void krakenX60_freeSpeedPositive() {
        assertTrue(MotorType.KRAKEN_X60.freeSpeedRPS() > 0,
                "Kraken X60 free speed should be positive");
    }

    @Test
    @DisplayName("FALCON_500: free speed is positive")
    void falcon500_freeSpeedPositive() {
        assertTrue(MotorType.FALCON_500.freeSpeedRPS() > 0,
                "Falcon 500 free speed should be positive");
    }

    @Test
    @DisplayName("freeSpeedRPS: derived from RPM correctly")
    void freeSpeedRPS_derivedCorrectly() {
        double rps = MotorType.KRAKEN_X60.freeSpeedRPS();
        double radPerSec = MotorType.KRAKEN_X60.freeSpeedRadPerSec();
        // rps * 2*pi = rad/s
        assertEquals(radPerSec, rps * 2 * Math.PI, EPSILON);
    }

    @Test
    @DisplayName("freeSpeedRadPerSec: returns positive value")
    void freeSpeedRadPerSec_positive() {
        assertTrue(MotorType.KRAKEN_X60.freeSpeedRadPerSec() > 0);
        assertTrue(MotorType.FALCON_500.freeSpeedRadPerSec() > 0);
    }

    // ===== DCMOTOR CREATION =====

    @Test
    @DisplayName("getDCMotor: returns valid DCMotor for Kraken X60")
    void getDCMotor_krakenX60_valid() {
        DCMotor motor = MotorType.KRAKEN_X60.getDCMotor(1);
        assertNotNull(motor, "DCMotor should not be null");
    }

    @Test
    @DisplayName("getDCMotor: returns valid DCMotor for Falcon 500")
    void getDCMotor_falcon500_valid() {
        DCMotor motor = MotorType.FALCON_500.getDCMotor(1);
        assertNotNull(motor, "DCMotor should not be null");
    }

    @Test
    @DisplayName("getDCMotor: multiple motors parameter")
    void getDCMotor_multipleMotors() {
        DCMotor single = MotorType.KRAKEN_X60.getDCMotor(1);
        DCMotor dual = MotorType.KRAKEN_X60.getDCMotor(2);
        assertNotNull(single);
        assertNotNull(dual);
    }

    // ===== PHYSICS CALCULATIONS =====

    @Test
    @DisplayName("maxMechanismRPM: reduced by gear ratio")
    void maxMechanismRPM_reducedByGearRatio() {
        double freeRPM = MotorType.KRAKEN_X60.freeSpeedRPS() * 60.0;
        double gearRatio = 10.0;
        double mechanismRPM = MotorType.KRAKEN_X60.maxMechanismRPM(gearRatio);
        assertEquals(freeRPM / gearRatio, mechanismRPM, EPSILON);
    }

    @Test
    @DisplayName("maxMechanismTorque: multiplied by gear ratio")
    void maxMechanismTorque_multipliedByGearRatio() {
        double gearRatio = 10.0;
        double torque1 = MotorType.KRAKEN_X60.maxMechanismTorque(gearRatio, 1);
        double torque2 = MotorType.KRAKEN_X60.maxMechanismTorque(gearRatio, 2);
        // Two motors should produce ~2x torque
        assertEquals(torque2, torque1 * 2.0, EPSILON);
    }

    @Test
    @DisplayName("holdingVoltage: returns positive value for positive load")
    void holdingVoltage_positiveForPositiveLoad() {
        double voltage = MotorType.KRAKEN_X60.holdingVoltage(1.0, 10.0);
        assertTrue(voltage > 0, "Holding voltage should be positive for positive load");
    }

    @Test
    @DisplayName("holdingVoltage: larger load requires more voltage")
    void holdingVoltage_largerLoadMoreVoltage() {
        double v1 = MotorType.KRAKEN_X60.holdingVoltage(1.0, 10.0);
        double v2 = MotorType.KRAKEN_X60.holdingVoltage(2.0, 10.0);
        assertTrue(v2 > v1, "Larger load should require more voltage");
    }
}
