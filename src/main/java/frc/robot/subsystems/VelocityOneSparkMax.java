// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityOneSparkMax extends SubsystemBase {
    private SparkMax motor;
    private double setSpeed;
    private String name;

    /**
     * Creates a positional control subsystem for a single REV Spark MAX motor.
     * @param motorID The CAN ID of the motor
     * @param name The name for SmartDashboard display
     * @param currentLimit The current limit for the motor
     * @param inverted Whether the motor should be inverted
     * @param brakeMode Whether the motor should use brake mode
     */
    /** Creates a new VelocityOneMotorRev. */
    public VelocityOneSparkMax(int motorID, String name, int currentLimit, boolean inverted, boolean brakeMode) {
        this.name = name;
        motor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(currentLimit)
              .inverted(inverted)
              .idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(config, null, PersistMode.kPersistParameters);
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
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(name + "_Speed", setSpeed);

    }
}
