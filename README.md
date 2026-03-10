# FrcCatalystTest

Integration test suite and example project for [FrcCatalyst](https://github.com/TomAs-1226/FrcCatalyst).

<p>
  <img src="https://img.shields.io/badge/WPILib-2026.2.1-green?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.1.1-orange?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-17-blue?style=flat-square&logo=openjdk" alt="Java 17"/>
  <img src="https://img.shields.io/badge/Tests-68%20passing-brightgreen?style=flat-square" alt="Tests"/>
</p>

---

## What Is This?

This project serves two purposes:

1. **Test Suite** — 68+ JUnit tests validating every FrcCatalyst component (utilities, hardware types, mechanisms, and the superstructure coordinator)
2. **Example Code** — A complete `RobotContainer` showing how to use every mechanism type, plus a standalone `BasicRobot` example for quick reference

## Prerequisites

- **WPILib 2026.2.1** installed
- **FrcCatalyst** published to local Maven (see below)
- **Java 17** (included with WPILib)

## Setup

### 1. Clone Both Repos

```bash
git clone https://github.com/TomAs-1226/FrcCatalyst.git
git clone https://github.com/TomAs-1226/FrcCatalystTest.git
```

### 2. Publish FrcCatalyst to Local Maven

```bash
cd FrcCatalyst
./gradlew publishToMavenLocal
```

This installs the library to your local Maven repository so the test project can find it.

### 3. Build the Test Project

```bash
cd ../FrcCatalystTest
./gradlew build
```

## Running Tests

### Unit Tests (Fast — No Hardware Needed)

```bash
./gradlew test
```

Runs **68 tests** covering:
- `CatalystMath` — deadband, square/cube input, angle math, physics helpers
- `InterpolatingTable` — interpolation, clamping, chaining
- `MovingAverage` — sliding window, reset, sample counting
- `FeedforwardGains` — elevator/arm/simple calculations
- `MotorType` — motor specs, DCMotor creation, physics calculations

These tests require **no robot hardware** and no WPILib HAL initialization. They run in pure Java.

### All Tests Including Mechanisms

```bash
./gradlew testAll
```

Runs everything above plus:
- `SlewRateLimiter` — rate limiting with HAL timer
- `TimedBoolean` — debouncing with HAL timer
- `LinearMechanismTest` — elevator construction, commands, triggers
- `RotationalMechanismTest` — arm construction, commands, triggers
- `FlywheelMechanismTest` — shooter velocity control, dual motors
- `RollerMechanismTest` — intake commands, game piece detection
- `WinchMechanismTest` — climber extend/retract, limits
- `SuperstructureCoordinatorTest` — state machine, transitions

> **Note:** Mechanism tests require WPILib simulation native libraries. They are excluded from the default `test` task because they need the HAL and CTRE Phoenix simulation runtime.

### Simulation

```bash
./gradlew simulateJava
```

Launches the WPILib Sim GUI with the full `RobotContainer` — all mechanisms simulated with accurate physics. Use the Sim Driver Station to enable teleop/auto modes.

## Project Structure

```
FrcCatalystTest/
├── src/
│   ├── main/java/frc/robot/
│   │   ├── Main.java                  # Entry point
│   │   ├── Robot.java                 # TimedRobot lifecycle
│   │   ├── RobotContainer.java        # Full integration test (all mechanisms)
│   │   └── examples/
│   │       └── BasicRobot.java        # Simple elevator + intake example
│   └── test/java/frc/lib/catalyst/
│       ├── hardware/
│       │   └── MotorTypeTest.java
│       ├── mechanisms/
│       │   ├── LinearMechanismTest.java
│       │   ├── RotationalMechanismTest.java
│       │   ├── FlywheelMechanismTest.java
│       │   ├── RollerMechanismTest.java
│       │   ├── WinchMechanismTest.java
│       │   └── SuperstructureCoordinatorTest.java
│       └── util/
│           ├── CatalystMathTest.java
│           ├── FeedforwardGainsTest.java
│           ├── InterpolatingTableTest.java
│           ├── MovingAverageTest.java
│           ├── SlewRateLimiterTest.java
│           └── TimedBooleanTest.java
├── build.gradle
├── vendordeps/
│   ├── Phoenix6.json
│   ├── PhotonVision.json
│   ├── PathplannerLib.json
│   └── WPILibNewCommands.json
└── README.md
```

## Example Code

### BasicRobot (Simple)

`src/main/java/frc/robot/examples/BasicRobot.java` — A minimal robot with:
- **Elevator** (LinearMechanism) — 4 named presets, manual jog, auto-zero
- **Intake** (RollerMechanism) — trigger-based intake/eject with game piece detection
- Clean operator bindings and a simple autonomous routine

### RobotContainer (Full Integration)

`src/main/java/frc/robot/RobotContainer.java` — Tests every FrcCatalyst feature:
- All 5 mechanism types (elevator, arm, wrist, shooter, intake, climber)
- SuperstructureCoordinator with 3 states
- InterpolatingTable for shooter distance lookup
- SlewRateLimiter, MovingAverage, TimedBoolean utilities
- CharacterizationHelper and MechanismVisualizer setup
- ProfiledPID alternative elevator

## Deploying to a Robot

To deploy this test project to an actual robot:

1. **Update team number** in `build.gradle`:
   ```gradle
   deploy {
       targets { roborio(getTargetTypeClass('RoboRIO')) {
           team = project.frc.getTeamOrDefault(YOUR_TEAM_NUMBER)
       }}
   }
   ```

2. **Update CAN IDs** in `RobotContainer.java` to match your hardware

3. **Deploy:**
   ```bash
   ./gradlew deploy
   ```

> **Warning:** The default `RobotContainer` creates motors on CAN IDs 1-10. Make sure these match your actual hardware before deploying to a real robot.

## Writing Your Own Tests

Use the existing tests as templates. Key patterns:

```java
// Pure utility test (no HAL needed)
@Test
void testSomething() {
    InterpolatingTable table = new InterpolatingTable()
        .add(1.0, 100)
        .add(3.0, 300);
    assertEquals(200.0, table.get(2.0), 0.01);
}

// HAL-dependent test
@BeforeAll
static void initHAL() {
    HAL.initialize(500, 0);
}

@Test
void testWithTimer() {
    SlewRateLimiter limiter = new SlewRateLimiter(10.0);
    double result = limiter.calculate(100.0);
    assertTrue(result < 100.0);
}
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `Could not find com.frccatalyst:FrcCatalyst:1.0.0` | Run `./gradlew publishToMavenLocal` in the FrcCatalyst directory first |
| JVM crash during mechanism tests | Use `./gradlew testAll` (not `test`) — mechanism tests need separate JVM config |
| `UnsatisfiedLinkError` on native libs | Make sure WPILib 2026 is installed and `WPILIB_HOME` is set |
| Tests pass locally but fail in CI | CI needs `chmod +x gradlew` and mechanism tests require simulation runtime |

## License

This project is available under the [MIT License](LICENSE).
