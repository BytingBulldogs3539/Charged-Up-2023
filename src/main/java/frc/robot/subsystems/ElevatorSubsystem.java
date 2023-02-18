// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX elevatorMotor;
  TalonSRX wrist;
  TalonFX elevatorRotationMotor;
  CANCoder elevatorRotationEncoder;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(IDConstants.ElevatorMotorID,IDConstants.ElevatorMotorCanName);
    elevatorRotationMotor =new TalonFX(IDConstants.ElevatorRotationMotorID, IDConstants.ElevatorRotationMotorCanName);
    elevatorRotationEncoder = new CANCoder(IDConstants.ElevatorRotationEncoderID);
    elevatorRotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    elevatorRotationMotor.configRemoteFeedbackFilter(IDConstants.ElevatorRotationEncoderID, RemoteSensorSource.CANCoder, 0);
    elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
    elevatorRotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
    wrist = new TalonSRX(IDConstants.WristMotorID);

    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    wrist.setNeutralMode(NeutralMode.Brake);
    elevatorRotationMotor.setNeutralMode(NeutralMode.Brake);

  }
  public void setGripperPositon(double x, double z){

  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

