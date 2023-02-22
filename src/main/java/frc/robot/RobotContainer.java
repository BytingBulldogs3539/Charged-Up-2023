// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DisableBreakMode;
import frc.robot.commands.FlipWrist;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetElevatorLengthCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.awt.geom.Point2D;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static IDConstants iDConstants = new IDConstants();
  public static DriveConstants driveConstants = new DriveConstants();
  public static ElevatorConstants elevatorConstants = new ElevatorConstants();

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController driverController = new CommandXboxController(1);
  public static CommandXboxController operatorController = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.start().onTrue(new ZeroGyroCommand());

    operatorController.leftTrigger().whileTrue(new IntakeCommand(1));

    operatorController.rightTrigger().whileTrue(new IntakeCommand(-1));

   /*  operatorController.a().onTrue(new SetElevatorLengthCommand(0));

    operatorController.b().onTrue(new SetElevatorLengthCommand(300));
    operatorController.y().onTrue(new SetElevatorLengthCommand(750));*/
    operatorController.a().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(79.0,0)));

    operatorController.rightBumper().onTrue(new FlipWrist());

    SmartDashboard.putData(new DisableBreakMode());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
