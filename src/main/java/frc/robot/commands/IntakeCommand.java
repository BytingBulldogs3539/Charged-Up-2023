// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  double intakeSpeed;
  boolean useSensor = true;
  Debouncer debouncer = new Debouncer(0.1,DebounceType.kBoth);

  public IntakeCommand(double speed) {
    this.intakeSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  public IntakeCommand(double speed, boolean useSensor) {
    this.intakeSpeed = speed;
    this.useSensor = useSensor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.setIntakeSpeed(intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When placing cube/cone, flash lights
    if (intakeSpeed > 0) {
      RobotContainer.ledSubsystem.flash();
    }

    // When intaking cube/cone
    else {
      // If using sensor, lights go solid when piece detected
      if (useSensor) {
        boolean pieceDetected = debouncer.calculate(RobotContainer.elevatorSubsystem.getIntakeSensor());
        if (pieceDetected) {
          RobotContainer.ledSubsystem.solid();
          // Cone detected in ground intake
          if (RobotContainer.elevatorSubsystem.getArmLevel() == Arm.groundIntake) {
            RobotContainer.elevatorSubsystem.setArmLevel(Arm.intake);
          }

        } else {
          // No piece detected, continue flashing
          RobotContainer.ledSubsystem.flash();
        }
      } else {
        // Not using sensor, always flash on intake
        RobotContainer.ledSubsystem.flash();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
    
    // If we were intaking a piece, set color to that piece
    if (intakeSpeed < 0) {
      RobotContainer.ledSubsystem.solid();
    }
    // If we were placing a piece, set back to default lights
    if (intakeSpeed > 0) {
      RobotContainer.ledSubsystem.setLEDs(LEDState.ON);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
