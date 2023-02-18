// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.control.Trajectory;

import java.awt.geom.Point2D;

import org.bytingbulldogs.bulldoglibrary.GearRatio;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorConstants;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ArmTrajectoryHandler;

public class ElevatorSubsystem extends SubsystemBase {

  public enum Arm {
    high, middle, low
  }

  public enum Wrist {
    cube, cone
  }

  public enum Sides {
    front, back
  }

  TalonFX elevatorMotor;
  TalonSRX wrist;
  CANCoder wristEncoder;
  TalonFX elevatorRotationMotor;
  CANCoder elevatorRotationEncoder;

  Arm armPosition = Arm.low;
  Sides side = Sides.front;
  Wrist wristOrrientation = Wrist.cube;

  ArmTrajectoryHandler trajectoryHandler = new ArmTrajectoryHandler(ElevatorConstants.maxArmVelocity,
      ElevatorConstants.maxArmAcceleration, 25, new Point2D.Double(0, 25));
  ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

  // private final TrapezoidProfile.Constraints m_constraints =
  // new TrapezoidProfile.Constraints(20, 20);
  // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  // private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  PIDController pid = new PIDController(ElevatorConstants.ElevatorRotationKp, ElevatorConstants.ElevatorRotationKi,
      ElevatorConstants.ElevatorRotationKd);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(IDConstants.ElevatorMotorID, IDConstants.ElevatorMotorCanName);
    elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor.setSelectedSensorPosition(0);
    elevatorMotor.setInverted(true);
    elevatorMotor.configForwardSoftLimitThreshold(ElevatorConstants.elevatorSoftMax);
    elevatorMotor.configReverseSoftLimitThreshold(ElevatorConstants.elevatorSoftMin);
    elevatorMotor.configForwardSoftLimitEnable(true);
    elevatorMotor.configReverseSoftLimitEnable(true);
    elevatorMotor.configSelectedFeedbackCoefficient(ElevatorConstants.ElevatorConversionRatio);
    elevatorMotor.config_kP(0, ElevatorConstants.ElevatorKp);
    elevatorMotor.config_kI(0, ElevatorConstants.ElevatorKi);
    elevatorMotor.config_kD(0, ElevatorConstants.ElevatorKd);

    elevatorMotor.configMotionCruiseVelocity(70);
    elevatorMotor.configMotionAcceleration(140);

    elevatorRotationEncoder = new CANCoder(IDConstants.ElevatorRotationEncoderID);
    elevatorRotationEncoder.configMagnetOffset(ElevatorConstants.ElevatorRotationMagnetOffset);
    elevatorRotationEncoder.configSensorDirection(false);
    elevatorRotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    elevatorRotationEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    elevatorRotationMotor = new TalonFX(IDConstants.ElevatorRotationMotorID,
        IDConstants.ElevatorRotationMotorCanName);
    elevatorRotationMotor.configRemoteFeedbackFilter(elevatorRotationEncoder, 0);
    elevatorRotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    elevatorRotationMotor.setNeutralMode(NeutralMode.Brake);
    elevatorRotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
    elevatorRotationMotor.configForwardSoftLimitThreshold(ElevatorConstants.elevatorRotationSoftMin);
    elevatorRotationMotor.configReverseSoftLimitThreshold(ElevatorConstants.elevatorRotationSoftMax);
    elevatorRotationMotor.configForwardSoftLimitEnable(true);
    elevatorRotationMotor.configReverseSoftLimitEnable(true);
    elevatorRotationMotor.setInverted(true);
    elevatorRotationMotor.setSensorPhase(true);
    elevatorRotationMotor.configSelectedFeedbackCoefficient(0.87890625); // Constant that defines .1degree resolution by
                                                                         // multiplying encoder ticks to get angle from
                                                                         // the cancoder then multiplies by 10
    elevatorRotationEncoder.setPosition(elevatorRotationEncoder.getAbsolutePosition());

    pid.enableContinuousInput(0, 360);

    wristEncoder = new CANCoder(IDConstants.WristEncoderID);
    wristEncoder.configMagnetOffset(ElevatorConstants.WristRotationMagnetOffset);

    wrist = new TalonSRX(IDConstants.WristMotorID);
    wrist.setNeutralMode(NeutralMode.Brake);
    wrist.configRemoteFeedbackFilter(wristEncoder, 0);
    wrist.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

    elevatorTab.addNumber("Arm X", () -> {
      return getGripperX();
    });
    elevatorTab.addNumber("Arm Y", () -> {
      return getGripperY();
    });

  }

  public void setArmRotation(double angle) {
    elevatorRotationMotor.set(ControlMode.Position, angle * 10);//Multiply by 10 to get .1 degree resolution.

  }

  public void setArmExtension(double length) {
    elevatorMotor.set(ControlMode.MotionMagic, length);
  }

  // public void setMotionProfile(double length) {
  // elevatorRotationMotor.set(ControlMode.Position, length *
  // ElevatorConstants.ElevatorConversionRatio);
  // }

  public void setWristRotation(double angle) {
    wrist.set(ControlMode.Position, angle);
  }

  public double getArmRotationError() {
    return Math.abs(elevatorRotationMotor.getClosedLoopError());
  }

  public double getArmExtensionError() {
    return Math.abs(elevatorRotationMotor.getClosedLoopError());
  }

  public double getElevatorLength() {
    return elevatorMotor.getSelectedSensorPosition() + ElevatorConstants.ElevatorMinExtension;
  }

  public double getElevatorRotationAngle() {
    return elevatorRotationEncoder.getAbsolutePosition();
  }

  public Point2D getGripperPositon() {
    return trajectoryHandler.polarToXY(getElevatorRotationAngle(), getElevatorLength());

  }

  public double getGripperX() {
    return getGripperPositon().getX();
  }

  public double getGripperY() {
    return getGripperPositon().getY();
  }

  public double calculateRotationFeedForward() {
    return Math.abs(getGripperPositon().getX()) * ElevatorConstants.ElevatorRotationFeedforwardRatio;
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Cube Mode", (wristOrrientation == Wrist.cube) ? true : false);
  }
}
