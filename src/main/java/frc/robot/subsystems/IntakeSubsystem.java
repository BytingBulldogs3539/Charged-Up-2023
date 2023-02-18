// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX intakeMotor;

  public IntakeSubsystem() {

    intakeMotor = new TalonSRX(IDConstants.IntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 60, 100));
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput,speed);
  }

  @Override
  public void periodic() {

  }
}
