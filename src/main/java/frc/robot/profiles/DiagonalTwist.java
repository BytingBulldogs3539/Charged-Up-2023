// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.profiles;

import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autoncommands.TrajectoryCommandGenerator;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.utilities.MPLoader;

public class DiagonalTwist extends SequentialCommandGroup {

    /*
     * DIRECTIONS: To load from a motion profile file, specify the name
     * of the file, use MPLoader to get the path data, and fill the
     * command sequence with the path data. The returned path data is
     * indexed as:
     *  [0]:     the pose command (must be used first)
     *  [1-n]:   each individual path in order
     */
    private final String filename = "diagonal_twist.txt";
    private Command[] paths = MPLoader.getCommandSequence(filename);
    private Command[] sequence = {
        new ZeroGyroCommand(0),
        new WaitCommand(.25),
        paths[0],
        paths[1],
        paths[2]
    };
    /*
     * No changes necessary below
     */

    public DiagonalTwist() { for (Command command : sequence) addCommands(command); }
}
