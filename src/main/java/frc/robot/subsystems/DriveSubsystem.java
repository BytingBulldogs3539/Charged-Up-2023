// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.SimplePathBuilder;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;
import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;
import frc.robot.autoncommands.TrajectoryCommandGenerator;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
	public enum StartPosition {
		RED_SMOOTH,
		RED_CABLE,
		BLUE_SMOOTH,
		BLUE_CABLE
	}

	public static final double MAX_VOLTAGE = 12.0;

	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			DriveConstants.wheelTrackWidth *
			DriveConstants.wheelDiameter * Math.PI;

	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(DriveConstants.wheelTrackWidth / 2.0,
					DriveConstants.wheelTrackWidth / 2.0);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DriveConstants.wheelTrackWidth / 2.0,
					DriveConstants.wheelTrackWidth / 2.0),
			// Front right
			new Translation2d(DriveConstants.wheelTrackWidth / 2.0,
					-DriveConstants.wheelTrackWidth / 2.0),
			// Back left
			new Translation2d(-DriveConstants.wheelTrackWidth / 2.0,
					DriveConstants.wheelTrackWidth / 2.0),
			// Back right
			new Translation2d(-DriveConstants.wheelTrackWidth / 2.0,
					-DriveConstants.wheelTrackWidth / 2.0));

	private final SwerveDrivePoseEstimator m_poseEstimator;

	private final Pigeon2 m_pigeon = new Pigeon2(IDConstants.PigeonID, IDConstants.PigeonCanName);

	Pose2d m_pose;

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private final ModuleConfiguration moduleGearRatio = new ModuleConfiguration(
			DriveConstants.wheelDiameter, DriveConstants.driveGearReduction, false,
			DriveConstants.steerGearReduction, false);

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");

	// Vision tracking
	AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonCamera leftCam;
	Transform3d robotToLeftCam = new Transform3d(
		new Translation3d(-0.1746 - .07 + 0.08, 0.2885 + 0.05, 0.3876),
		new Rotation3d(Math.toRadians(0), 0, Math.toRadians(4))
	);
	
	public PhotonCamera rightCam;
	Transform3d robotToRightCam = new Transform3d(
		new Translation3d(-0.1746 - .07 + 0.09, -0.2885 - 0.01, 0.3876),
		new Rotation3d(Math.toRadians(0), 0, Math.toRadians(-4))
	);
	
	PhotonPoseEstimator leftPhotonPoseEstimator;
	PhotonPoseEstimator rightPhotonPoseEstimator;
	boolean useVision = false;

	// Vision auto-alignment
	Command autoAlign;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

		} catch (Exception e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		leftCam = new PhotonCamera("LeftCam");
		leftPhotonPoseEstimator = new PhotonPoseEstimator(
			aprilTagFieldLayout,
			PoseStrategy.MULTI_TAG_PNP,
			leftCam,
			robotToLeftCam
		);
		rightCam = new PhotonCamera("RightCam");
		rightPhotonPoseEstimator = new PhotonPoseEstimator(
			aprilTagFieldLayout,
			PoseStrategy.MULTI_TAG_PNP,
			rightCam,
			robotToRightCam
		);

		setGyroscope(180);

		m_frontLeftModule = SdsSwerveModuleHelper.createFalcon500(
			// This parameter is optional, but will allow you to see the current state of
			// the module on the dashboard.
			driveTrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(0, 0),
			// This can either be STANDARD or FAST depending on your gear configuration
			moduleGearRatio,
			// This is the ID of the drive motor
			IDConstants.FLDriveID,
			IDConstants.FLDriveCanName,
			// This is the ID of the steer motor
			IDConstants.FLSteeringID,
			IDConstants.FLSteeringCanName,
			// This is the ID of the steer encoder
			IDConstants.FLCanCoderID,
			IDConstants.FLEncoderCanName,
			// This is how much the steer encoder is offset from true zero (In our case,
			// zero is facing straight forward)
			DriveConstants.FLSteerOffset
		);

		// We will do the same for the other modules
		m_frontRightModule = SdsSwerveModuleHelper.createFalcon500(
			driveTrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(2, 0),
			moduleGearRatio,
			IDConstants.FRDriveID,
			IDConstants.FRDriveCanName,
			IDConstants.FRSteeringID,
			IDConstants.FRSteeringCanName,
			IDConstants.FRCanCoderID,
			IDConstants.FREncoderCanName,
			DriveConstants.FRSteerOffset
		);

		m_backLeftModule = SdsSwerveModuleHelper.createFalcon500(
			driveTrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(4, 0),
			moduleGearRatio,
			IDConstants.BLDriveID,
			IDConstants.BLDriveCanName,
			IDConstants.BLSteeringID,
			IDConstants.BLSteeringCanName,
			IDConstants.BLCanCoderID,
			IDConstants.BLEncoderCanName,
			DriveConstants.BLSteerOffset
		);

		m_backRightModule = SdsSwerveModuleHelper.createFalcon500(
			driveTrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
				.withSize(2, 4)
				.withPosition(6, 0),
			moduleGearRatio,
			IDConstants.BRDriveID,
			IDConstants.BRDriveCanName,
			IDConstants.BRSteeringID,
			IDConstants.BRSteeringCanName,
			IDConstants.BRCanCoderID,
			IDConstants.BREncoderCanName,
			DriveConstants.BRSteerOffset
		);

		m_poseEstimator = new SwerveDrivePoseEstimator(
			m_kinematics,
			getGyroscopeRotation(),
			getModulePositions(),
			new Pose2d(0, 0, new Rotation2d()),
			VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0)),
			VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10))
		);

		m_pose = m_poseEstimator.update(getGyroscopeRotation(), getModulePositions());
		driveTrainTab.addNumber("X Pose", m_pose::getX);
		driveTrainTab.addNumber("Y Pose", m_pose::getY);
		driveTrainTab.addNumber("Theta Pose", () -> (m_pose.getRotation().getDegrees()));
		driveTrainTab.addNumber("Pitch", this::getPitch);

		setDefaultCommand(new DriveCommand(this));
	}

	public void setVisionWeights(double visionX, double visionY, int visionDeg) {
		m_poseEstimator.setVisionMeasurementStdDevs(
			VecBuilder.fill(visionX, visionY, Units.degreesToRadians(visionDeg))
		);
	}

	public void setLeftCamera(boolean on) {
		if (DriverStation.getAlliance() == Alliance.Red)
			aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
		else
			aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
		
		leftCam.setDriverMode(false);
		leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
	}

	public void setRightCamera(boolean on) {
		if (DriverStation.getAlliance() == Alliance.Red)
			aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
		else
			aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

		rightCam.setDriverMode(false);
		rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
	}

	public void setStartPosition(StartPosition position)
	{
		// Configure which camera to use in auton based on start position
		switch (position) {
			case RED_SMOOTH:
				aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				rightCam.setDriverMode(false);
				leftCam.setDriverMode(true);
				break;
			case RED_CABLE:
				aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				rightCam.setDriverMode(true);
				leftCam.setDriverMode(false);
				break;
			case BLUE_SMOOTH:
				aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				rightCam.setDriverMode(true);
				leftCam.setDriverMode(false);
				break;
			case BLUE_CABLE:
				aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				rightCam.setDriverMode(false);
				leftCam.setDriverMode(true);
				break;
		}
		// Update the april tag layout based on our start position
		leftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
		rightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
	}

	/**
	 * Used to zero the pigeon / gyroscope.
	 */
	public void setGyroscope(double angle) {
		m_pigeon.setYaw(angle);
	}

	/**
	 * Used to get the angle of rotation of the robot from the last time the gyro
	 * was zeroed.
	 * 
	 * @returns gyro angle.
	 */
	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(m_pigeon.getYaw());
	}

	public double getPitch() {
		return m_pigeon.getRoll();
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public Pose2d getPose() {
		return m_pose;
	}

	public void resetPose(Pose2d pose) {
		m_poseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = {
			m_frontLeftModule.getPosition(),
			m_frontRightModule.getPosition(),
			m_backLeftModule.getPosition(),
			m_backRightModule.getPosition()
		};
		return positions;
	}

	public Optional<EstimatedRobotPose> getEstimatedLeftGlobalPose(Pose2d prevEstimatedRobotPose) {
		leftPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return leftPhotonPoseEstimator.update();
	}
	public Optional<EstimatedRobotPose> getEstimatedRightGlobalPose(Pose2d prevEstimatedRobotPose) {
		rightPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return rightPhotonPoseEstimator.update();
	}
	
	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

	@Override
	public void periodic() {

		Optional<EstimatedRobotPose> resultLeft = getEstimatedLeftGlobalPose(
			m_poseEstimator.getEstimatedPosition()
		);
		Optional<EstimatedRobotPose> resultRight = getEstimatedRightGlobalPose(
			m_poseEstimator.getEstimatedPosition()
		);

		if (useVision) {
			if (resultLeft.isPresent()) {
				EstimatedRobotPose camPoseLeft = resultLeft.get();
				m_poseEstimator.addVisionMeasurement(
					camPoseLeft.estimatedPose.toPose2d(), camPoseLeft.timestampSeconds
				);
				SmartDashboard.putNumber("LeftCam X Pos", camPoseLeft.estimatedPose.toPose2d().getX());
				SmartDashboard.putNumber("LeftCam Y Pos", camPoseLeft.estimatedPose.toPose2d().getY());
			}
			if (resultRight.isPresent()) {
				EstimatedRobotPose camPoseRight = resultRight.get();
				m_poseEstimator.addVisionMeasurement(
					camPoseRight.estimatedPose.toPose2d(), camPoseRight.timestampSeconds
				);
				SmartDashboard.putNumber("RightCam X Pos", camPoseRight.estimatedPose.toPose2d().getX());
				SmartDashboard.putNumber("RightCam Y Pos", camPoseRight.estimatedPose.toPose2d().getY());
			}
		}

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		//System.out.println(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);

		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());

		m_pose = m_poseEstimator.update(getGyroscopeRotation(), getModulePositions());

		SmartDashboard.putNumber("Pose X", getPose().getX());
		SmartDashboard.putNumber("Pose Y", getPose().getY());

	}
}
