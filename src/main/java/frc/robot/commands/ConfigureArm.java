// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Sides;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;

public class ConfigureArm extends CommandBase {
  /** Creates a new SetArmHeight. */
  Sides side;
  Arm position;
  Wrist orientation;
  public ConfigureArm(Sides side, Arm position, Wrist orientation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.side = side;
    this.position = position;
    this.orientation = orientation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorSubsystem.setSide(side);
    RobotContainer.elevatorSubsystem.setArmLevel(position);
    RobotContainer.elevatorSubsystem.setWristOrientationOverride(orientation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
