// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.SetArmSide;
import frc.robot.commands.SetWristOrientationOverride;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Sides;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMidCone extends SequentialCommandGroup {
  /** Creates a new Place_High_Cone. */
  public PlaceMidCone() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetArmSide(Sides.front), new SetArmHeight(Arm.middle), new SetWristOrientationOverride(Wrist.cone),
        new WaitCommand(4), new IntakeCommand(1).withTimeout(3),new SetArmSide(Sides.front), new SetArmHeight(Arm.intake), new WaitCommand(.5),new SetWristOrientationOverride(Wrist.cube));
  }
}
