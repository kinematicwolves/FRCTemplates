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
public class PositionalOneSparkMax extends SubsystemBase {
    private SparkMax motor;
    private SparkClosedLoopController pidController;
    private double setPoint;
    private double encoderTolerance;
    private String name;

    /**
     * Creates a positional control subsystem for a single REV Spark MAX motor.
     * @param motorID The CAN ID of the motor
     * @param encoderTolerance The tolerance for checking position
     * @param name The name for SmartDashboard display
     * @param currentLimit The current limit for the motor
     * @param inverted Whether the motor should be inverted
     * @param brakeMode Whether the motor should use brake mode
     * @param kp Proportional gain for PID
     * @param ki Integral gain for PID
     * @param kd Derivative gain for PID
     */
    public PositionalOneSparkMax(int motorID, double encoderTolerance, String name, int currentLimit, boolean inverted, boolean brakeMode, double kp, double ki, double kd) {
        this.encoderTolerance = encoderTolerance;
        this.name = name;
        motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);
        pidController = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(currentLimit)
              .inverted(inverted)
              .idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast)
              .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
        motor.configure(config, null, PersistMode.kPersistParameters);

    }

    /**
     * Sets the motor to a target position.
     * @param position The desired position in encoder units
     */
    public void setPosition(double position) {
        setPoint = position;
        pidController.setReference(position, ControlType.kPosition);
    }

    /**
     * Checks if the motor is within the set position tolerance.
     * @return True if within tolerance, false otherwise
     */
    public boolean atPosition() {
        double currentPos = motor.getEncoder().getPosition();
        return (currentPos >= setPoint - encoderTolerance) && (currentPos <= setPoint + encoderTolerance);
    }

    /**
     * Runs the motor at a specified output power.
     * @param outputFraction Motor power from -1.0 to 1.0
     */
    public void setSpeed(double outputFraction) {
        motor.set(outputFraction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(name + "_SetPoint", setPoint);
        SmartDashboard.putNumber(name + "_Pose", motor.getEncoder().getPosition());
        SmartDashboard.putBoolean(name + "_AtPose", atPosition());
    }
}
