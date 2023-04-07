// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveSubsystem sub;
  double speed = -1;

  public AutoBalance(DriveSubsystem sub,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub = sub;
    addRequirements(sub);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d gyroAngle = sub.getGyroscopeRotation();
    if (sub.getPitch() > 9.0) {
      sub.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            -speed,
            0,
            0,
            gyroAngle));

    } else if(sub.getPitch() < -9.0) {
      sub.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speed,
              0,
              0,
              gyroAngle));
    }
    else {
      sub.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0,
              0,
              gyroAngle));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
