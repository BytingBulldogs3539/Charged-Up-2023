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
import java.awt.geom.Point2D;

import org.bytingbulldogs.bulldoglibrary.GearRatio;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorConstants;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;
import frc.robot.utilities.ArmTrajectoryHandler;

public class ElevatorSubsystem extends SubsystemBase {

	enum Arm {
		high, middle, low
	}

	enum Wrist {
		cube, cone
	}

	enum Sides {
		front, back
	}

	TalonFX elevatorMotor;
	TalonSRX wrist;
	TalonFX elevatorRotationMotor;
	CANCoder elevatorRotationEncoder;
	Arm armPosition = Arm.low;
	Sides side = Sides.front;
	Wrist wristOrrientation = Wrist.cube;
	ArmTrajectoryHandler trajectoryHandler = new ArmTrajectoryHandler(ElevatorConstants.maxArmVelocity,
			ElevatorConstants.maxArmAcceleration);
	ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {
		elevatorMotor = new TalonFX(IDConstants.ElevatorMotorID, IDConstants.ElevatorMotorCanName);
		elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
		elevatorMotor.setNeutralMode(NeutralMode.Coast);
		elevatorMotor.setSelectedSensorPosition(0);// -235359u
		elevatorMotor.setInverted(true);

		elevatorRotationEncoder = new CANCoder(IDConstants.ElevatorRotationEncoderID);
		elevatorRotationEncoder.configMagnetOffset(ElevatorConstants.ElevatorRotationMagnetOffset);

		elevatorRotationMotor = new TalonFX(IDConstants.ElevatorRotationMotorID,
				IDConstants.ElevatorRotationMotorCanName);
		elevatorRotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		elevatorRotationMotor.configRemoteFeedbackFilter(IDConstants.ElevatorRotationEncoderID,
				RemoteSensorSource.CANCoder,
				0);
		elevatorRotationMotor.setNeutralMode(NeutralMode.Coast);
		elevatorRotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));

		wrist = new TalonSRX(IDConstants.WristMotorID);
		wrist.setNeutralMode(NeutralMode.Brake);

	}

	public void setGripperPositon(Arm arm, Wrist wrist, Sides side) {
		this.armPosition = arm;
		this.wristOrrientation = wrist;
		this.side = side;
	}

	public double getElevatorLength() {
		return elevatorMotor.getSelectedSensorPosition() * ElevatorConstants.ElevatorConversionRatio;
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

	@Override
	public void periodic() {
		elevatorTab.addNumber("Arm X", () -> {
			return getGripperX();
		});
		elevatorTab.addNumber("Arm Y", () -> {
			return getGripperY();
		});
		SmartDashboard.putBoolean("Cube Mode", (wristOrrientation == Wrist.cube) ? true : false);

	}
}
