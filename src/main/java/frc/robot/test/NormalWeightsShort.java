// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

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
import frc.robot.autoncommands.SetStartPosition;
import frc.robot.autoncommands.SetPoseCommand;
import frc.robot.autoncommands.TrajectoryCommandGenerator;
import frc.robot.commands.ConfigureArm;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.SetArmSide;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.SetVision;
import frc.robot.commands.SetVisionWeights;
import frc.robot.commands.SetWristOrientationOverride;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DriveSubsystem.StartPosition;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Sides;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.MPLoader;

public class NormalWeightsShort extends SequentialCommandGroup {

    /*
     * DIRECTIONS: To load from a motion profile file, specify the name
     * of the file, use MPLoader to get the path data, and fill the
     * command sequence with the path data. The returned path data is
     * indexed as:
     *  [0]:     the pose command (must be used first)
     *  [1-n]:   each individual path in order
     */
    private final String filename = "three_piece_red.txt";
    private Command[] paths = MPLoader.getCommandSequence(filename);
    private Command[] sequence = {
        // Setup
        new ZeroGyroCommand(180),
        new SetVision(true),
        new SetVisionWeights(0.5, 0.5, 10),
        new SetLEDs(LEDState.CONE),
        new SetStartPosition(StartPosition.RED_SMOOTH),
        new WaitCommand(.1),
        // Place cone
        // new ConfigureArm(Sides.front, Arm.high, Wrist.cone),
        new WaitCommand(1.75),
        new IntakeCommand(1).withTimeout(0.6),
        // Drive to first cube
        new ParallelCommandGroup(
            new SequentialCommandGroup(
            //    new ConfigureArm(Sides.back, Arm.intake, Wrist.cube),
               new WaitCommand(1),
               new SetVision(false)
            ),
            new SequentialCommandGroup(
                new WaitCommand(0.9),
                new IntakeCommand(1).withTimeout(2.4)
            ),
            new SequentialCommandGroup(
                new WaitCommand(.01),
                paths[0],
                paths[1]
            )
        ),
        // Place first cube
        // new ConfigureArm(Sides.front, Arm.high, Wrist.cube),
        new ParallelCommandGroup(
            paths[2],
            new SequentialCommandGroup(
                new WaitCommand(2.2),
                new IntakeCommand(-1).withTimeout(0.5)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new SetVision(true)
            )
        ),
        // Drive to second cube
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                paths[3]
            ),
            new SequentialCommandGroup(
                // new ConfigureArm(Sides.back, Arm.intake, Wrist.cube),
                new WaitCommand(1),
                new SetVision(false),
                new IntakeCommand(1).withTimeout(2)
            )
        ),
        // Place second cube
        new ParallelCommandGroup(
            paths[4],
            new SequentialCommandGroup(
                // new ConfigureArm(Sides.front, Arm.middle, Wrist.cube),
                new WaitCommand(2.5),
                // new SetArmHeight(Arm.intake),
                new IntakeCommand(-1).withTimeout(1)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1.7),
                new SetVision(true)
            )
        )
    };
    /*
     * No changes necessary below
     */

    public NormalWeightsShort() { for (Command command : sequence) addCommands(command); }
}
