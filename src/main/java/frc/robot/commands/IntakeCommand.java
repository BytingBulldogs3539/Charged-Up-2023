// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem.Arm;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  double speed;

  public IntakeCommand(double speed) {
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.setIntakeSpeed(speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(speed < 0)
    {
      if(RobotContainer.elevatorSubsystem.getArmLevel() == Arm.groundIntake)
      {
        if(RobotContainer.elevatorSubsystem.getIntakeSensor()) {
          RobotContainer.elevatorSubsystem.setArmLevel(Arm.intake);
        }
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
