// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX rotationMotor;
  private CANSparkMax intakeMotor;
  
  public IntakeSubsystem() {
rotationMotor = new TalonSRX(IDConstants.RotationMotorID);

intakeMotor = new CANSparkMax(IDConstants.IntakeMotorID, MotorType.kBrushless);

intakeMotor.setSmartCurrentLimit(30);
rotationMotor.enableCurrentLimit(true);

}

public void setIntakeSpeed(double speed){
intakeMotor.set(speed);
}


  @Override
  public void periodic() {
    
  }
}
