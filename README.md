# FrcCatalystTest

Integration test suite and example project for [FrcCatalyst](https://github.com/TomAs-1226/FrcCatalyst).

<p>
  <img src="https://img.shields.io/badge/FrcCatalyst-v1.3.2-e94560?style=flat-square" alt="FrcCatalyst"/>
  <img src="https://img.shields.io/badge/WPILib-2026.2.1-green?style=flat-square" alt="WPILib"/>
  <img src="https://img.shields.io/badge/Phoenix%206-26.1.1-orange?style=flat-square" alt="Phoenix 6"/>
  <img src="https://img.shields.io/badge/Java-17-blue?style=flat-square&logo=openjdk" alt="Java 17"/>
  <img src="https://img.shields.io/badge/Tests-passing-brightgreen?style=flat-square" alt="Tests"/>
</p>

---

## What Is This?

This project serves two purposes:

1. **Test Suite** — JUnit tests validating every FrcCatalyst component (utilities, hardware types, all mechanisms, and the modern `Superstructure` state machine)
2. **Example Code** — A complete `RobotContainer` that drives every mechanism through one `Superstructure` state machine, plus a standalone `BasicRobot` example for quick reference

## Prerequisites

- **WPILib 2026.2.1** installed
- **Internet connection** (FrcCatalyst v1.3.2 is pulled from JitPack on the first build)
- **Java 17** (included with WPILib)

## Setup

### 1. Clone this repo

```bash
git clone https://github.com/TomAs-1226/FrcCatalystTest.git
```

### 2. Build it

```bash
cd FrcCatalystTest
./gradlew build
```

That's it. FrcCatalyst v1.3.2 comes from JitPack automatically, so there is nothing else to install.

<details><summary>Rather build FrcCatalyst from source?</summary>

Clone the library, publish it to your local Maven, then change the dependency in `build.gradle`
from the JitPack coordinate to `com.frccatalyst:FrcCatalyst:1.3.2`:

```bash
git clone https://github.com/TomAs-1226/FrcCatalyst.git
cd FrcCatalyst
./gradlew publishToMavenLocal
```
</details>

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
- `ClawMechanismTest` — gripper commands, game-piece detection (new in 1.2)
- `DifferentialWristMechanismTest` — pitch + roll construction, commands, ranges (new in 1.2)
- `ServoMechanismTest` — PWM servo commands, angle clamping, named positions (new in 1.3)
- `SuperstructureTest` — the modern `Superstructure` state machine: graph, transitions, `explain()`

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
│       │   ├── ClawMechanismTest.java
│       │   ├── DifferentialWristMechanismTest.java
│       │   ├── ServoMechanismTest.java
│       │   └── SuperstructureTest.java
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

`src/main/java/frc/robot/RobotContainer.java` — Exercises every FrcCatalyst feature:
- Eight mechanisms bound into one `Superstructure` state machine: elevator, arm, differential wrist, shooter, intake, claw, climber, and a servo hood
- A six-state graph (STOW / INTAKE / CARRY / AIM / SCORE / CLIMB) with a staged edge (elevator up before the arm swings), an entry guard (no SCORE without a game piece), and a global interlock (only STOW/CLIMB while the winch is extended)
- `explain()` printed on init and on a button — the "why is it stuck?" dump
- InterpolatingTable, SlewRateLimiter, MovingAverage, TimedBoolean utilities
- CharacterizationHelper and MechanismVisualizer setup
- ProfiledPID alternative elevator (standalone, not in the state machine)

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
| `Could not find com.github.TomAs-1226:FrcCatalyst:v1.3.2` | Check your internet connection. JitPack builds the library on first request, so the very first build can take a minute. Or build from source (see Setup). |
| JVM crash during mechanism tests | Use `./gradlew testAll` (not `test`) — mechanism tests need separate JVM config |
| `UnsatisfiedLinkError` on native libs | Make sure WPILib 2026 is installed and `WPILIB_HOME` is set |
| Tests pass locally but fail in CI | CI needs `chmod +x gradlew` and mechanism tests require simulation runtime |

## License

This project is available under the [MIT License](LICENSE).
