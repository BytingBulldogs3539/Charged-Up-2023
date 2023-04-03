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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.IDConstants;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
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

	AprilTagFieldLayout aprilTagFieldLayout;



	// Forward Camera
	PhotonCamera cam;
	Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

	// Construct PhotonPoseEstimator
	PhotonPoseEstimator photonPoseEstimator;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
			
		} catch (Exception e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam,
					robotToCam);

		setGyroscope(0);

		m_frontLeftModule = SdsSwerveModuleHelper.createFalcon500(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				driveTrainTab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
						0),
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
				DriveConstants.FLSteerOffset);

		// We will do the same for the other modules
		m_frontRightModule = SdsSwerveModuleHelper.createFalcon500(
				driveTrainTab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,
						0),
				moduleGearRatio,
				IDConstants.FRDriveID,
				IDConstants.FRDriveCanName,
				IDConstants.FRSteeringID,
				IDConstants.FRSteeringCanName,
				IDConstants.FRCanCoderID,
				IDConstants.FREncoderCanName,
				DriveConstants.FRSteerOffset);

		m_backLeftModule = SdsSwerveModuleHelper.createFalcon500(
				driveTrainTab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,
						0),
				moduleGearRatio,
				IDConstants.BLDriveID,
				IDConstants.BLDriveCanName,
				IDConstants.BLSteeringID,
				IDConstants.BLSteeringCanName,
				IDConstants.BLCanCoderID,
				IDConstants.BLEncoderCanName,
				DriveConstants.BLSteerOffset);

		m_backRightModule = SdsSwerveModuleHelper.createFalcon500(
				driveTrainTab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,
						0),
				moduleGearRatio,
				IDConstants.BRDriveID,
				IDConstants.BRDriveCanName,
				IDConstants.BRSteeringID,
				IDConstants.BRSteeringCanName,
				IDConstants.BRCanCoderID,
				IDConstants.BREncoderCanName,
				DriveConstants.BRSteerOffset);

		m_poseEstimator = new SwerveDrivePoseEstimator(
				m_kinematics,
				getGyroscopeRotation(), getModulePositions(),
				new Pose2d(0, 0, new Rotation2d()),
				VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(3)),
				VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

		m_pose = m_poseEstimator.update(getGyroscopeRotation(), getModulePositions());
		driveTrainTab.addNumber("X Pose", m_pose::getX);
		driveTrainTab.addNumber("Y Pose", m_pose::getY);
		driveTrainTab.addNumber("Theta Pose", () -> (m_pose.getRotation().getDegrees()));
		driveTrainTab.addNumber("Pitch", this::getPitch);

		setDefaultCommand(new DriveCommand(this));
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
		SwerveModulePosition[] positions = { m_frontLeftModule.getPosition(),
				m_frontRightModule.getPosition(),
				m_backLeftModule.getPosition(), m_backRightModule.getPosition() };
		return positions;
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update();
	}

	@Override
	public void periodic() {

		Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
		if (result.isPresent()) {
			EstimatedRobotPose camPose = result.get();
			m_poseEstimator.addVisionMeasurement(
					camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
		}

		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());

		m_pose = m_poseEstimator.update(getGyroscopeRotation(), getModulePositions());

	}
}
