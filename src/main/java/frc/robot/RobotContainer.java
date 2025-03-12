// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SetPositionOneSparkMax;
import frc.robot.commands.SetPositionTwoSparkMax;
import frc.robot.commands.SetSpeedOneSparkMax;
import frc.robot.subsystems.PositionalOneSparkMax;
import frc.robot.subsystems.PositionalTwoSparkMax;
import frc.robot.subsystems.VelocityOneSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final PositionalTwoSparkMax elevator = new PositionalTwoSparkMax(5, 6, 0.5, "elevator", 40, false, true, false, 0.1, 0, 0);
    private final PositionalOneSparkMax wrist = new PositionalOneSparkMax(7, 0.5, "wrist", 40, false, false, 0.1, 0.0, 0.0);
    private final VelocityOneSparkMax roller = new VelocityOneSparkMax(8, "Intake", 20, false, false);

    // create your commands here
    private final SetPositionOneSparkMax moveWristToReady =new SetPositionOneSparkMax(wrist, 1);
    private final SetPositionTwoSparkMax moveElevatorToScoringPose = new SetPositionTwoSparkMax(elevator, 10);
    private final SetPositionTwoSparkMax moveElevatorToHome = new SetPositionTwoSparkMax(elevator, 0);
    private final SetSpeedOneSparkMax intake = new SetSpeedOneSparkMax(roller, 0.5);
    private final SetSpeedOneSparkMax outtake = new SetSpeedOneSparkMax(roller, -0.5);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController opController = new CommandXboxController(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        opController.leftBumper().whileTrue(intake);
        opController.leftBumper().whileTrue(outtake);
        opController.a()
            .onTrue(moveElevatorToScoringPose.andThen(moveWristToReady))
            .onFalse(moveElevatorToHome);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new InstantCommand();
    }
}
