// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.profiles;

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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.SetArmSide;
import frc.robot.commands.SetWristOrientationOverride;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Sides;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.MPLoader;

public class TwoPieceLeft extends SequentialCommandGroup {

    /*
     * DIRECTIONS: To load from a motion profile file, specify the name
     * of the file, use MPLoader to get the path data, and fill the
     * command sequence with the path data. The returned path data is
     * indexed as:
     *  [0]:     the pose command (must be used first)
     *  [1-n]:   each individual path in order
     */
    private final String filename = "two_piece_left.txt";
    private Command[] paths = MPLoader.getCommandSequence(filename);
    private Command[] sequence = {
        // Setup
        new ZeroGyroCommand(180),
        new WaitCommand(.25),
        // Place cone
        new SetArmSide(Sides.front),
        new SetArmHeight(Arm.high),
        new SetWristOrientationOverride(Wrist.cone),
        new WaitCommand(3),
        new IntakeCommand(1).withTimeout(0.8),
        new SetArmSide(Sides.front),
        new SetArmHeight(Arm.intake),
        // Drive to second piece
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(.5),
                new SetWristOrientationOverride(Wrist.cube)
            ),
            new SequentialCommandGroup(
                new WaitCommand(.01),
                paths[0],
                paths[1]
            )
        ),
        new ParallelCommandGroup(
            new SetArmHeight(Arm.groundIntake),
            paths[2],
            new IntakeCommand(-1,false).withTimeout(1.5)
        ),
        new SetArmHeight(Arm.intake),
        new ParallelCommandGroup(
            paths[3],
            new SequentialCommandGroup(
                new IntakeCommand(-1,false).withTimeout(1),
                new SetArmHeight(Arm.middle),
                new SetWristOrientationOverride(Wrist.cone)
            )
        ),
        new IntakeCommand(1).withTimeout(0.8)
    };
    /*
     * No changes necessary below
     */

    public TwoPieceLeft() { for (Command command : sequence) addCommands(command); }
}
