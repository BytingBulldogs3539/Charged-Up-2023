// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
public static final double MAX_VOLTAGE = 12.0;

public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			RobotContainer.constants.getDriveConstants().getWheelTrackWidth() *
			RobotContainer.constants.getDriveConstants().getWheelDiameter() * Math.PI;

	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(RobotContainer.constants.getDriveConstants().getWheelTrackWidth() / 2.0,
					RobotContainer.constants.getDriveConstants().getWheelBase() / 2.0);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(RobotContainer.constants.getDriveConstants().getWheelTrackWidth() / 2.0,
					RobotContainer.constants.getDriveConstants().getWheelBase() / 2.0),
			// Front right
			new Translation2d(RobotContainer.constants.getDriveConstants().getWheelTrackWidth() / 2.0,
					-RobotContainer.constants.getDriveConstants().getWheelBase() / 2.0),
			// Back left
			new Translation2d(-RobotContainer.constants.getDriveConstants().getWheelTrackWidth() / 2.0,
					RobotContainer.constants.getDriveConstants().getWheelBase() / 2.0),
			// Back right
			new Translation2d(-RobotContainer.constants.getDriveConstants().getWheelTrackWidth() / 2.0,
					-RobotContainer.constants.getDriveConstants().getWheelBase() / 2.0));

	SwerveDriveOdometry m_odometry;

	private final Pigeon2 m_pigeon = new Pigeon2(RobotContainer.constants.getDriveConstants().getPigeonID());

	Pose2d m_pose;

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
			  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
