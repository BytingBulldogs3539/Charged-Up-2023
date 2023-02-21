// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import java.awt.geom.Point2D;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorConstants;
import frc.robot.IDConstants;
import frc.robot.commands.DisableBreakMode;
import frc.robot.utilities.ArmPosition;
import frc.robot.utilities.ArmTrajectoryGenerator;
import frc.robot.utilities.ArmTrajetoryFollower;


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
	PIDController m_rController;
	PIDController m_eController;

	Arm armPosition = Arm.low;
	Sides side = Sides.front;
	Wrist wristOrrientation = Wrist.cube;

	ArmTrajectoryGenerator trajectoryHandler;

	ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {
		elevatorMotor = new TalonFX(IDConstants.ElevatorMotorID, IDConstants.ElevatorMotorCanName);
		elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
		elevatorMotor.setNeutralMode(NeutralMode.Brake);
		elevatorMotor.setSelectedSensorPosition(0);
		elevatorMotor.setInverted(true);
		elevatorMotor.configForwardSoftLimitThreshold(ElevatorConstants.elevatorSoftMax*10);
		elevatorMotor.configReverseSoftLimitThreshold(ElevatorConstants.elevatorSoftMin*10);
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
		elevatorRotationMotor.configSelectedFeedbackCoefficient(0.87890625); // Constant that defines .1degree
																				// resolution by
																				// multiplying encoder ticks to get
																				// angle from
																				// the cancoder then multiplies by 10
		elevatorRotationMotor.configMotionCruiseVelocity(10);
		elevatorRotationMotor.configMotionAcceleration(100);

		elevatorRotationEncoder.setPosition(elevatorRotationEncoder.getAbsolutePosition());

		wristEncoder = new CANCoder(IDConstants.WristEncoderID);
		wristEncoder.configMagnetOffset(ElevatorConstants.WristRotationMagnetOffset);

		wrist = new TalonSRX(IDConstants.WristMotorID);
		wrist.setNeutralMode(NeutralMode.Brake);
		wrist.configRemoteFeedbackFilter(wristEncoder, 0);
		wrist.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

		elevatorTab.addNumber("Arm X", this::getGripperX);
		elevatorTab.addNumber("Arm Y", this::getGripperY);
		m_eController = new PIDController(ElevatorConstants.ElevatorKp, ElevatorConstants.ElevatorKi, ElevatorConstants.ElevatorKd);
		m_rController = new PIDController(ElevatorConstants.ElevatorRotationKp, ElevatorConstants.ElevatorRotationKi, ElevatorConstants.ElevatorRotationKd);
	
		trajectoryHandler = new ArmTrajectoryGenerator(ElevatorConstants.maxArmVelocity,
			ElevatorConstants.maxArmAcceleration,ElevatorConstants.ElevatorMinExtension, new Point2D.Double(0,ElevatorConstants.ElevatorMaxExtension), new Point2D.Double(0, ElevatorConstants.ElevatorMaxExtension));
	}

	/*public void setArmExtension(double length) {
		elevatorMotor.set(ControlMode.MotionMagic, length);
	}*/

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
		return (elevatorMotor.getSelectedSensorPosition()/10.0) + ElevatorConstants.ElevatorMinExtension;
	}

	public Rotation2d getElevatorRotationAngle() {
		return Rotation2d.fromDegrees(elevatorRotationEncoder.getAbsolutePosition());
	}

	public Point2D.Double getGripperPositon() {
		return ArmTrajectoryGenerator.polarToXY(new ArmPosition(getElevatorRotationAngle(), getElevatorLength()));
	}

	public double getGripperX() {
		return getGripperPositon().getX();
	}

	public double getGripperY() {
		return getGripperPositon().getY();
	}

	public double calculateRotationFeedForward() {
		return getGripperPositon().getX() * ElevatorConstants.ElevatorRotationFeedforwardRatio;
	}

	public ArmPosition getArmPose() {

		return new ArmPosition(getElevatorRotationAngle(), getElevatorLength());
	}

	public void setExtensionSpeed(double speed) {
		elevatorMotor.set(ControlMode.PercentOutput, speed);
	}

	public void setRotationSpeed(double speed) {
		elevatorRotationMotor.set(ControlMode.PercentOutput, speed);
	}

	public Command getArmTrajectoryFollower(Point2D.Double endPoint) {

		return new ArmTrajetoryFollower(endPoint, trajectoryHandler, this::getArmPose, this::getGripperPositon,
				this::setExtensionSpeed, this::setRotationSpeed, m_rController, getElevatorLength(), m_eController, ElevatorConstants.ElevatorMinExtension, ElevatorConstants.ElevatorMaxExtension, Rotation2d.fromDegrees(ElevatorConstants.elevatorRotationSoftMin/10.0),
				Rotation2d.fromDegrees(ElevatorConstants.elevatorRotationSoftMax/10.0));
	}
	public void setBreakMode(boolean enabled){
		if (enabled){
			elevatorRotationMotor.setNeutralMode(NeutralMode.Brake);
			elevatorMotor.setNeutralMode(NeutralMode.Brake);
		}
		else{
			elevatorRotationMotor.setNeutralMode(NeutralMode.Coast);
			elevatorMotor.setNeutralMode(NeutralMode.Coast);
		}
	}
	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Cube Mode", (wristOrrientation == Wrist.cube) ? true : false);
	}
}
