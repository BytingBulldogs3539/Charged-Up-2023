// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX elevatorMotor;
  TalonFX elevatorRotationMotor;
  CANCoder elevatorRotationEncoder;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(RobotContainer.iDConstants.ElevatorMotorID,RobotContainer.iDConstants.ElevatorMotorCanName);
    elevatorRotationMotor =new TalonFX(RobotContainer.iDConstants.ElevatorRotationMotorID, RobotContainer.iDConstants.ElevatorRotationMotorCanName);
    elevatorRotationEncoder = new CANCoder(RobotContainer.iDConstants.);
    elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
    elevatorRotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
