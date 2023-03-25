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

public class Square1M extends SequentialCommandGroup {

    private final String filename = "1m_square.txt";

    private Command[] sequence = {
        new ZeroGyroCommand(0),
        new WaitCommand(.25),
        null,
        null,
        null,
        null,
        null
    };

    public Square1M() {
        try {
            for (Command command : MPLoader.getCommandSequence(sequence, filename)) {
                addCommands(command);
            }
        }
        catch (java.io.IOException e) {

        }
    }
}
