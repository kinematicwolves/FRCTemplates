# FRCTemplates

This repo contains example subsystems that are the bare minimum commonly used in FRC robots. These include:

- PositionalWithOneMotor
  - Example: Controlling a wrist or elevator to a position with one motor
- PositionalWithTwoMotors
  - Example: Controlling a wrist or elevator to a position with two motors
- VelocityWithOneMotor
  - Example: Running an intake or roller with one motor
- VelocityWithTwoMotors
  - Example: Running counter-rotating rollers off of two motors

The repo includes examples for both the REV Spark Max and CTRE TalonFX motor controllers.

The intent of this repo is to provide bare-bones, no-frills, get-a-team-running-now code. It's not the most elegant, it's not necessarily the safest (e.g., no soft limits), but it is designed to be simple, readable, and fast to get going. This code is meant to be easily enhanced or adjusted as needed.

## Usage

- Clone this repo
- Create a branch off of main
- Merge in the needed subsystems
- Create the subsystems in the robot container:
  - Populate CAN IDs, current limits, subsystem names, motor inversions, etc.
  - It is recommended to start with motors in coast mode.
- Quick tune:
  - Subsystems should print out basic info to the SmartDashboard.
  - Use this information to inform setpoints, PID tuning, etc.

### Subsystem Names

Subsystem class names follow the format of control type, number of motors, and motor driver type.

### Name Format

<Velocity / Positional><One / Two><SparkMax / TalonFX>

- [Velocity / Position]:
  - Use a Velocity subsystem to control a roller, simple intake, etc., where you only need to spin a motor(s) at a set speed.
  - Use a Position subsystem to control a wrist, elevator, etc., where you need to control the position of a motor/motors.
- [One / Two]:
  - Use a One subsystem when there is only one motor in your subsystem.
  - Use a Two subsystem when there are two motors that must spin together.
- [SparkMax / TalonFX]:
  - Use a SparkMax subsystem to control a motor connected to a SparkMax controller.
  - Use a TalonFX subsystem to control a Falcon, Kraken, or similar motor.

### Example

- For an elevator with two Neos controlled by SparkMaxes, use PositionTwoMotorSparkMax.
- For an intake with a single Neo 550 controlled by a SparkMax, use VelocityOneMotorSparkMax.

## Pre-Requisites

- CAN IDs must be set.
- For SparkMax controllers, the motor type should be configured and updated using the REV Hardware Client.
- For TalonFX controllers, the motor type should be configured and updated using Phoenix Tuner X.

## Limitations

- This repo is not intended to be a fully functional solution, but rather provide boilerplate code to get teams up and running quickly.
- Additional motor types and controllers may be added in the future, but there are no guarantees.
- SparkMax controllers are currently only supported when wired through CAN.
- This code is provided with no guarantees of functionality. It is the responsibility of the user to ensure the robot is safe at all times when operating or testing the robot.
