// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.05);

		// Square the axis
		value = Math.copySign(value * value, value);

		return value;
	}
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d gyroAngle = RobotContainer.driveSubsystem.getGyroscopeRotation();
    double translationXPercent = modifyAxis(RobotContainer.driverController.getLeftX());
		double translationYPercent = -modifyAxis(RobotContainer.driverController.getLeftY());
		double rotationPercent = -modifyAxis(RobotContainer.driverController.getRightY());

    double driveSpeedMultiplier = RobotContainer.driveConstants.driveSpeedMultiplier;
    double rotationSpeedMultiplier = RobotContainer.driveConstants.rotationSpeedMultiplier;


    //If the driver is pressing the right trigger then FULL SPEED.
    if(RobotContainer.driverController.getRightTriggerAxis()>.1)
		{
			driveSpeedMultiplier = 1.0;
		}

    if (RobotContainer.driverController.rightBumper().getAsBoolean()) {
				
			gyroAngle = Rotation2d.fromDegrees(0);
		}


    RobotContainer.driveSubsystem.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						driveSpeedMultiplier * translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
						driveSpeedMultiplier * translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
						rotationSpeedMultiplier * rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
						gyroAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
