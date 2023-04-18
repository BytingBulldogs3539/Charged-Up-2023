// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.SimplePathBuilder;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;
import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.autoncommands.TrajectoryCommandGenerator;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class AutoPoleAlign extends CommandBase {
    /** Creates a new SetArmHeight. */

    Command autoAlign;
	public final double FORWARD_DISTANCE = 1.7;
	public final double[] RED_POLES = {
		7.51, 6.40, 5.84, 4.72, 4.16, 3.05
	};
	public final double[] BLUE_POLES = {
		0.52, 1.63, 2.19, 3.31, 3.87, 4.98 
	};

    public AutoPoleAlign() {
        double[] poles = DriverStation.getAlliance() == Alliance.Red?
            RED_POLES : BLUE_POLES;
        double robotY = RobotContainer.driveSubsystem.getPose().getY();
        double nearestY = -1;
        double smallestDist = 9999;

        // Find the nearest pole Y coordinate
        for (double y : poles) {
            if (Math.abs(robotY - y) < smallestDist) {
                smallestDist = Math.abs(robotY - y);
                nearestY = y;
            }
        }

        this.autoAlign = TrajectoryCommandGenerator.getMotionCommand(
			new SimplePathBuilder(
                new Vector2(
                    RobotContainer.driveSubsystem.getPose().getX(),
                    RobotContainer.driveSubsystem.getPose().getY()),
                Rotation2.fromDegrees(
                    RobotContainer.driveSubsystem.getPose().getRotation().getDegrees()
                ))
				.lineTo(
					new Vector2(FORWARD_DISTANCE, nearestY),
					Rotation2.fromDegrees(180))
				.build(),
			new TrajectoryConstraint[] {
				(TrajectoryConstraint) new MaxAccelerationConstraint(1),
				(TrajectoryConstraint) new MaxVelocityConstraint(1)
			},
			RobotContainer.driveSubsystem
		);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.ledSubsystem.saveState();
        RobotContainer.ledSubsystem.setLEDs(LEDState.CLIMBING);

        autoAlign.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!this.autoAlign.isFinished()) this.autoAlign.cancel();
        RobotContainer.ledSubsystem.restoreState();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.autoAlign.isFinished();
    }
}
