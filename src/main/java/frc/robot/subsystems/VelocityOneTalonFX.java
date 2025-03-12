// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityOneTalonFX extends SubsystemBase {
    private TalonFX motor;
    public TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    
    private double rollerSpeed = 0;
    
    /** Creates a new Gripper. */
    /**
     * Creates a positional control subsystem for a single CTRE TalonFX motor.
     * @param motorID The CAN ID of the motor
     * @param name The name for SmartDashboard display
     * @param currentLimit The current limit for the motor
     * @param inverted Whether the motor should be inverted
     * @param brakeMode Whether the motor should use brake mode
     */
    public VelocityOneTalonFX(int motorID, String name, int currentLimit, boolean inverted, boolean brakeMode) {
        motor = new TalonFX(motorID);
        /* Factory Reset */
        motor.getConfigurator().apply(new TalonFXConfiguration());

        /* Current Limit */
        motorConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;

        /* Inversion Factor */
        if (inverted)
          motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        /* Idle Mode */
        if (brakeMode)
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        else
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Save Config */
        motor.getConfigurator().refresh(motorConfig);
    }

    /**
     * User sets the motor speed
     * 
     * @param speed commanded output value, -1 to 1
     */
    public void setSpeed(double speed) {
        this.rollerSpeed = speed;
        motor.set(speed);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Gripper speed", this.rollerSpeed);
   }
}