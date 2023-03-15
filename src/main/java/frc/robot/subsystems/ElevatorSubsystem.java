// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorConstants;
import frc.robot.IDConstants;
import frc.robot.utilities.ArmPosition;
import frc.robot.utilities.ArmTrajectoryGenerator;
import frc.robot.utilities.ArmTrajetoryFollower;

public class ElevatorSubsystem extends SubsystemBase {

	public enum Arm {
		high, middle, low, intake, HumanPlayer
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

	Arm armPosition = Arm.intake;
	Sides side = Sides.front;
	private Wrist wristOrrientation = Wrist.cube;

	ArmTrajectoryGenerator trajectoryHandler;

	ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {
		elevatorMotor = new TalonFX(IDConstants.ElevatorMotorID, IDConstants.ElevatorMotorCanName);
		elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 100));
		elevatorMotor.setNeutralMode(NeutralMode.Brake);
		elevatorMotor.setSelectedSensorPosition(0);
		elevatorMotor.setInverted(true);
		elevatorMotor.configForwardSoftLimitThreshold((ElevatorConstants.elevatorSoftMax * 10));
		elevatorMotor.configReverseSoftLimitThreshold(ElevatorConstants.elevatorSoftMin * 10);
		elevatorMotor.configForwardSoftLimitEnable(true);
		elevatorMotor.configReverseSoftLimitEnable(true);
		elevatorMotor.configSelectedFeedbackCoefficient(ElevatorConstants.ElevatorConversionRatio);
		elevatorMotor.config_kP(0, ElevatorConstants.ElevatorKp);
		elevatorMotor.config_kI(0, ElevatorConstants.ElevatorKi);
		elevatorMotor.config_kD(0, ElevatorConstants.ElevatorKd);

		elevatorMotor.configMotionCruiseVelocity(70);
		elevatorMotor.configMotionAcceleration(140);
		//elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		//elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

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
		wristEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

		wrist = new TalonSRX(IDConstants.WristMotorID);
		wrist.setNeutralMode(NeutralMode.Brake);
		wrist.configRemoteFeedbackFilter(wristEncoder, 0);
		wrist.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
		wrist.config_kP(0, ElevatorConstants.WristKp);
		wrist.config_kI(0, ElevatorConstants.WristKi);
		wrist.config_kD(0, ElevatorConstants.WristKd);
		wrist.configSelectedFeedbackCoefficient(0.087890625);
		wristEncoder.setPosition(wristEncoder.getAbsolutePosition());

		elevatorTab.addNumber("Arm X", this::getGripperX);
		elevatorTab.addNumber("Arm Y", this::getGripperY);
		elevatorTab.addNumber("Elevator Angle", () -> {
			return getElevatorRotationAngle().getDegrees();
		});
		m_eController = new PIDController(ElevatorConstants.ElevatorKp, ElevatorConstants.ElevatorKi,
				ElevatorConstants.ElevatorKd);
		m_rController = new PIDController(ElevatorConstants.ElevatorRotationKp, ElevatorConstants.ElevatorRotationKi,
				ElevatorConstants.ElevatorRotationKd);

		trajectoryHandler = new ArmTrajectoryGenerator(ElevatorConstants.maxArmVelocity,
				ElevatorConstants.maxArmAcceleration, ElevatorConstants.ElevatorMinExtension);

		wrist.set(ControlMode.Position, wrist.getSelectedSensorPosition());

		setDefaultCommand(new ArmTrajetoryFollower(this::getTargetPosition, trajectoryHandler, this::getArmPose,
				this::getGripperPositon,
				this::setExtensionSpeed, this::setRotationSpeed, m_rController,
				ElevatorConstants.ElevatorRotationFeedforwardRatio, m_eController,
				ElevatorConstants.ElevatorMinExtension, ElevatorConstants.ElevatorMaxExtension,
				Rotation2d.fromDegrees(ElevatorConstants.elevatorRotationSoftMin / 10.0),
				Rotation2d.fromDegrees(ElevatorConstants.elevatorRotationSoftMax / 10.0), this));
	}

	public void setWristOrientation(Wrist orientation) {
		if (getElevatorRotationAngle().getDegrees() > ElevatorConstants.IntakeLimitMax) {
			this.wristOrrientation = orientation;
		}
	}

	public void setWristOrientationOverride(Wrist orientation) {
		this.wristOrrientation = orientation;
	}
	

	public void setSide(Sides side) {
		this.side = side;
	}

	public Wrist getWristOrientation() {
		return wristOrrientation;
	}

	public Sides getSide() {
		return side;
	}

	public void setArmLevel(Arm arm) {
		this.armPosition = arm;
	}

	public Arm getArmLevel() {
		return armPosition;
	}

	public double getElevatorLength() {
		return (elevatorMotor.getSelectedSensorPosition() / 10.0) + ElevatorConstants.ElevatorMinExtension;
	}

	public Rotation2d getElevatorRotationAngle() {
		return Rotation2d.fromDegrees(elevatorRotationEncoder.getPosition());
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

	public ArmPosition getArmPose() {

		return new ArmPosition(getElevatorRotationAngle(), getElevatorLength());
	}

	public void setExtensionSpeed(double speed) {

		elevatorMotor.set(ControlMode.PercentOutput, speed);
		SmartDashboard.putNumber("Elevator Speed Follower Output", speed);
	}

	public void setRotationSpeed(double speed) {
		SmartDashboard.putNumber("Rotation Speed Follower Output", speed);
		if (speed > .35) {
			speed = .35;
		}
		if (speed < -.35) {
			speed = -.35;
		}
		elevatorRotationMotor.set(ControlMode.PercentOutput, speed);
	}

	public void setBreakMode(boolean enabled) {
		if (enabled) {
			elevatorRotationMotor.setNeutralMode(NeutralMode.Brake);
			elevatorMotor.setNeutralMode(NeutralMode.Brake);
		} else {
			elevatorRotationMotor.setNeutralMode(NeutralMode.Coast);
			elevatorMotor.setNeutralMode(NeutralMode.Coast);
		}
	}

	public Point2D.Double getTargetPosition() {
		Point2D.Double pos = new Point2D.Double(0,0);
		if (side == Sides.front) {
			if (wristOrrientation == Wrist.cone) {
				if (armPosition == Arm.intake) {
					pos = new Point2D.Double(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Point2D.Double(ElevatorConstants.frontConeLowX, ElevatorConstants.frontConeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Point2D.Double(ElevatorConstants.frontConeMidX, ElevatorConstants.frontConeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Point2D.Double(ElevatorConstants.frontConeHighX, ElevatorConstants.frontConeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Point2D.Double(ElevatorConstants.frontConeHumanPlayerX,
							ElevatorConstants.frontConeHumanPlayerY);
				}
			} else if (wristOrrientation == Wrist.cube) {
				if (armPosition == Arm.intake) {
					pos = new Point2D.Double(ElevatorConstants.frontCubeIntakeX, ElevatorConstants.frontCubeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Point2D.Double(ElevatorConstants.frontCubeLowX, ElevatorConstants.frontCubeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Point2D.Double(ElevatorConstants.frontCubeMidX, ElevatorConstants.frontCubeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Point2D.Double(ElevatorConstants.frontCubeHighX, ElevatorConstants.frontCubeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Point2D.Double(ElevatorConstants.frontCubeHumanPlayerX,
							ElevatorConstants.frontCubeHumanPlayerY);
				}
			}
		} else if (side == Sides.back) {
			if (wristOrrientation == Wrist.cone) {
				if (armPosition == Arm.intake) {
					pos = new Point2D.Double(ElevatorConstants.backConeIntakeX, ElevatorConstants.backConeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Point2D.Double(ElevatorConstants.backConeLowX, ElevatorConstants.backConeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Point2D.Double(ElevatorConstants.backConeMidX, ElevatorConstants.backConeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Point2D.Double(ElevatorConstants.backConeHighX, ElevatorConstants.backConeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Point2D.Double(ElevatorConstants.backConeHumanPlayerX,
							ElevatorConstants.backConeHumanPlayerY);
				}
			} else if (wristOrrientation == Wrist.cube) {
				if (armPosition == Arm.intake) {
					pos = new Point2D.Double(ElevatorConstants.backCubeIntakeX, ElevatorConstants.backCubeIntakeY);
				} else if (armPosition == Arm.low) {
					pos = new Point2D.Double(ElevatorConstants.backCubeLowX, ElevatorConstants.backCubeLowY);
				} else if (armPosition == Arm.middle) {
					pos = new Point2D.Double(ElevatorConstants.backCubeMidX, ElevatorConstants.backCubeMidY);
				} else if (armPosition == Arm.high) {
					pos = new Point2D.Double(ElevatorConstants.backCubeHighX, ElevatorConstants.backCubeHighY);
				} else if (armPosition == Arm.HumanPlayer) {
					pos = new Point2D.Double(ElevatorConstants.backCubeHumanPlayerX,
							ElevatorConstants.backCubeHumanPlayerY);
				}
			}
		}
		if(pos.x == 0 && pos.y == 0){
		pos = new Point2D.Double(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY);
		}
		return pos;
	}

	@Override
	public void periodic() {



		SmartDashboard.putBoolean("Cube Mode", (wristOrrientation == Wrist.cube) ? true : false);
		if (getElevatorRotationAngle().getDegrees() > ElevatorConstants.IntakeLimitMax) {
			if (wristOrrientation == Wrist.cube && side == Sides.front) {
				wrist.set(ControlMode.Position, 0);
			}
			if (wristOrrientation == Wrist.cube && side == Sides.back) {
				wrist.set(ControlMode.Position, 180);
			}
			if (wristOrrientation == Wrist.cone && side == Sides.back) {
				wrist.set(ControlMode.Position, 0);
			}
			if (wristOrrientation == Wrist.cone && side == Sides.front) {
				wrist.set(ControlMode.Position, 180);
			}
		}
	}
}
