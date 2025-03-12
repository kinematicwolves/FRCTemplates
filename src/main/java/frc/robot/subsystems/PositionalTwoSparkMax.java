// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Positional Subsystem with One REV Motor
public class PositionalTwoSparkMax extends SubsystemBase {
    private SparkMax leaderMotor;
    private SparkMax followerMotor;
    private SparkClosedLoopController leaderPIDController;
    private SparkClosedLoopController followerPIDController;
    private double setPoint;
    private double encoderTolerance;
    private String name;

    /**
     * Creates a positional control subsystem for a single REV Spark MAX motor.
     * @param leaderMotorID The CAN ID of the lead motor
     * @param followerMotorID The CAN ID of the following motor
     * @param encoderTolerance The tolerance for checking position
     * @param name The name for SmartDashboard display
     * @param currentLimit The current limit for the motor
     * @param leaderInverted Whether the lead motor should be inverted
     * @param followerInverted Whether the follower motor should be inverted
     * @param brakeMode Whether the motor should use brake mode
     * @param kp Proportional gain for PID
     * @param ki Integral gain for PID
     * @param kd Derivative gain for PID
     */
    public PositionalTwoSparkMax(int leaderMotorID, int followerMotorID, double encoderTolerance, String name, int currentLimit, boolean leaderInverted, boolean followerInverted, boolean brakeMode, double kp, double ki, double kd) {
        this.encoderTolerance = encoderTolerance;
        this.name = name;

        // setup the leader motor
        leaderMotor = new SparkMax(leaderMotorID, SparkLowLevel.MotorType.kBrushless);
        leaderPIDController = leaderMotor.getClosedLoopController();

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.smartCurrentLimit(currentLimit)
              .inverted(leaderInverted)
              .idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
              .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
        leaderMotor.configure(leaderConfig, null, PersistMode.kPersistParameters);

        // setup the follower motor
        followerMotor = new SparkMax(followerMotorID, SparkLowLevel.MotorType.kBrushless);
        followerPIDController = followerMotor.getClosedLoopController();

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.smartCurrentLimit(currentLimit)
              .inverted(followerInverted)
              .idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
              .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
        followerMotor.configure(followerConfig, null, PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor to a target position.
     * @param position The desired position in encoder units
     */
    public void setPosition(double position) {
        setPoint = position;
        leaderPIDController.setReference(position, ControlType.kPosition);
        followerPIDController.setReference(position, ControlType.kPosition);
    }

    /**
     * Checks if the motor is within the set position tolerance.
     * @return True if within tolerance, false otherwise
     */
    public boolean atPosition() {
        double currentPos = leaderMotor.getEncoder().getPosition();
        return (currentPos >= setPoint - encoderTolerance) && (currentPos <= setPoint + encoderTolerance);
    }

    /**
     * Runs the motor at a specified output power.
     * @param outputFraction Motor power from -1.0 to 1.0
     */
    public void setSpeed(double outputFraction) {
        leaderMotor.set(outputFraction);
        followerMotor.set(outputFraction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(name + "_SetPoint", setPoint);
        SmartDashboard.putNumber(name + "_Pose", leaderMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean(name + "_AtPose", atPosition());
    }
}
