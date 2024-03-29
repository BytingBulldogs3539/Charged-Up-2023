// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.SimplePathBuilder;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;
import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autoncommands.SetPoseCommand;
import frc.robot.autoncommands.TrajectoryCommandGenerator;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ConfigureArm;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.SetArmSide;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.SetWristOrientationOverride;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Sides;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;
import frc.robot.subsystems.LEDSubsystem.LEDState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHighConeAndBalance extends SequentialCommandGroup {
    /** Creates a new Place_High_Cone. */
    public PlaceHighConeAndBalance() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ZeroGyroCommand(180),
            new SetLEDs(LEDState.CONE),
            new WaitCommand(.25),
            new SetPoseCommand(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
            new ConfigureArm(Sides.front, Arm.high, Wrist.cone),
            new WaitCommand(2),
            new IntakeCommand(1).withTimeout(0.5),
            new SetArmSide(Sides.front),
            new SetArmHeight(Arm.intake),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new SetWristOrientationOverride(Wrist.cube)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(.01),
                    TrajectoryCommandGenerator.getMotionCommand(
                        new SimplePathBuilder(
                            new Vector2(0, 0),
                            Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(1, 0), Rotation2.fromDegrees(135))
                        .lineTo(new Vector2(4.5, 0), Rotation2.fromDegrees(135))
                        .build(),
                        getSlowConstraints(),
                        RobotContainer.driveSubsystem
                    ),
                    TrajectoryCommandGenerator.getMotionCommand(
                        new SimplePathBuilder(
                            new Vector2(4.5, 0),
                            Rotation2.fromDegrees(135))
                        .lineTo(new Vector2(2.8, 0), Rotation2.fromDegrees(135))
                        .lineTo(new Vector2(2.1, 0), Rotation2.fromDegrees(180))
                        .build(),
                        getSlowConstraints(),
                        RobotContainer.driveSubsystem
                    ),
                    new AutoBalance(RobotContainer.driveSubsystem,1.5).withTimeout(5)
                )
            )
        );
    }

    public TrajectoryConstraint[] getSlowConstraints() {
        TrajectoryConstraint[] constraints = { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
            (TrajectoryConstraint) new MaxVelocityConstraint(1) };
        return constraints;
    }

    public TrajectoryConstraint[] getConstraints() {
        TrajectoryConstraint[] constraints = { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
            (TrajectoryConstraint) new MaxVelocityConstraint(2) };
        return constraints;
    }
}