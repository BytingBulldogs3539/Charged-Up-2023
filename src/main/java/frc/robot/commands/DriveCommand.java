// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.swervedrivespecialties.swervelib.control.PidConstants;
import com.swervedrivespecialties.swervelib.control.PidController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Arm;;

public class DriveCommand extends CommandBase {
    /** Creates a new DriveCommand. */
    private PidController rotationController;

    public DriveCommand(DriveSubsystem subsystem) {
        addRequirements(subsystem);
        rotationController = new PidController(
            new PidConstants(DriveConstants.RotationkP, DriveConstants.RotationkI, DriveConstants.RotationkD)
        );
        rotationController.setInputRange(-Math.PI, Math.PI);
        rotationController.setOutputRange(-1, 1);
        rotationController.setContinuous(true);
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
    private static double modifyTurnAxis(double value) {
      // Deadband
      value = deadband(value, 0.05);

      return value;
  }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Axis modifier function
        value = Math.copySign(value * value * value, value);
  
        return value;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double driveSpeedMultiplier = DriveConstants.driveSpeedMultiplier;
        double rotationSpeedMultiplier = DriveConstants.rotationSpeedMultiplier;

        // Right trigger: full speed
        if (RobotContainer.driverController.getRightTriggerAxis() > .1) {
            driveSpeedMultiplier = 1.0;
        }
        // Left trigger: slow mode
        else if (RobotContainer.driverController.getLeftTriggerAxis() > .1) {
            driveSpeedMultiplier = 0.1;
            rotationSpeedMultiplier = 0.1;
        }
        // Arm on ground: slow mode
        else if (RobotContainer.elevatorSubsystem.getArmLevel()==Arm.groundIntake) {
            driveSpeedMultiplier = 0.15;
            rotationSpeedMultiplier = 0.1;
        }

        Rotation2d gyroAngle = RobotContainer.driveSubsystem.getGyroscopeRotation();
        double translationXPercent = -modifyAxis(RobotContainer.driverController.getLeftY());
        double translationYPercent = -modifyAxis(RobotContainer.driverController.getLeftX());
        double rotationPercent = rotationSpeedMultiplier * -modifyTurnAxis(RobotContainer.driverController.getRightX());

        if (RobotContainer.driverController.a().getAsBoolean()) {
            Double setPoint = 0.0;
            if (Math.abs(gyroAngle.getRadians()) > Math.PI / 2.0) {
                setPoint = Math.PI;
            }
            rotationController.setSetpoint(setPoint);
            rotationPercent = rotationController.calculate(gyroAngle.getRadians(), .02) / 2.0;
        }

        if (RobotContainer.driverController.rightBumper().getAsBoolean()) {
            gyroAngle = Rotation2d.fromDegrees(0);
        }

        RobotContainer.driveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveSpeedMultiplier * translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                driveSpeedMultiplier * translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                gyroAngle
            )
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveSubsystem.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
