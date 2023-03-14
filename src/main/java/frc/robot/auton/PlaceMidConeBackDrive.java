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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMidConeBackDrive extends SequentialCommandGroup {
  /** Creates a new Place_High_Cone. */
  public PlaceMidConeBackDrive() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ZeroGyroCommand(180),new WaitCommand(.25), new SetPoseCommand(new Pose2d(0,0, Rotation2d.fromDegrees(180))), new SetArmSide(Sides.front), new SetArmHeight(Arm.middle), new SetWristOrientationOverride(Wrist.cone),
        new WaitCommand(4), new IntakeCommand(1).withTimeout(1.5),new SetArmSide(Sides.front), new SetArmHeight(Arm.intake), new WaitCommand(.5),new SetWristOrientationOverride(Wrist.cube)
        ,TrajectoryCommandGenerator.getMotionCommand(
          new SimplePathBuilder(
            new Vector2(0, 0), Rotation2.fromDegrees(180))
            .lineTo(new Vector2(3.9,0),Rotation2.fromDegrees(180)).build()
            , getConstraints(), RobotContainer.driveSubsystem));
  }
  public TrajectoryConstraint[] getConstraints() {
    TrajectoryConstraint[] constraints = { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
        (TrajectoryConstraint) new MaxVelocityConstraint(2) };
    return constraints;
  }
}
