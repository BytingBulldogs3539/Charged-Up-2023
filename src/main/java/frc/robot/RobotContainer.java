// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.DriveTest;
import frc.robot.auton.PlaceHighCone;
import frc.robot.auton.PlaceHighConeAndBalance;
import frc.robot.auton.PlaceHighConeBackDrive;
import frc.robot.auton.PlaceHighConeBalanceOld;
import frc.robot.auton.PlaceHighCube;
import frc.robot.auton.PlaceHighCubeAndBalance;
import frc.robot.auton.PlaceHighCubeBackDrive;
import frc.robot.auton.PlaceMidCone;
import frc.robot.auton.PlaceMidConeAndBalance;
import frc.robot.auton.PlaceMidConeBackDrive;
import frc.robot.auton.PlaceMidCube;
import frc.robot.auton.PlaceMidCubeAndBalance;
import frc.robot.auton.PlaceMidCubeBackDrive;
import frc.robot.profiles.MPTest;
import frc.robot.profiles.Square1M;
import frc.robot.profiles.ForwardTest;
import frc.robot.profiles.ForwardBackTest;
import frc.robot.profiles.CurveRightTest;
import frc.robot.profiles.Diagonal;
import frc.robot.profiles.DiagonalTwist;
import frc.robot.profiles.TwoConeBlue;
import frc.robot.profiles.TwoConeBlueTest;
import frc.robot.profiles.TwoConeRed;
import frc.robot.profiles.TwoConeRedTest;
import frc.robot.profiles.L;
import frc.robot.commands.DisableBreakMode;
import frc.robot.commands.FlipArmSideCommand;
import frc.robot.commands.FlipWrist;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmHeight;
import frc.robot.commands.SetConeLights;
import frc.robot.commands.SetCubeLights;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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

  public SendableChooser<Command> chooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    PortForwarder.add(5800, "photonvision.local", 5800);
    PortForwarder.add(1181, "photonvision.local", 1181);
    PortForwarder.add(1182, "photonvision.local", 1182);
    PortForwarder.add(1183, "photonvision.local", 1183);
    PortForwarder.add(1184, "photonvision.local", 1184);
    configureBindings();
    putAuton();
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
    driverController.start().onTrue(new ZeroGyroCommand(0));

    operatorController.leftTrigger().whileTrue(new IntakeCommand(1,false));

    operatorController.rightTrigger().whileTrue(new IntakeCommand(-1,false));

    operatorController.leftBumper().onTrue(new FlipArmSideCommand());

    driverController.y().onTrue(new SetConeLights());

    driverController.x().onTrue(new SetCubeLights());

    operatorController.a().onTrue(new SetArmHeight(Arm.intake));
    operatorController.b().onTrue(new SetArmHeight(Arm.low));
    operatorController.y().onTrue(new SetArmHeight(Arm.middle));
    operatorController.x().onTrue(new SetArmHeight(Arm.high));
    operatorController.povRight().onTrue(new SetArmHeight(Arm.HumanPlayer));
    operatorController.povDown().onTrue(new SetArmHeight(Arm.groundIntake));

    //operatorController.a().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(38.926915,-70.0)));
    //operatorController.b().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(110.0,-1)));
    //operatorController.x().onTrue(elevatorSubsystem.getArmTrajectoryFollower(new Point2D.Double(-60.57,-78.12)));

    operatorController.rightBumper().onTrue(new FlipWrist());

    SmartDashboard.putData(new DisableBreakMode());
  }

  public void putAuton() {
		chooser = new SendableChooser<Command>();
    chooser.addOption("TestDrive", new DriveTest());
		chooser.setDefaultOption("Place Cone High", new PlaceHighCone()); 
    chooser.addOption("Place Cone High and Balance", new PlaceHighConeAndBalance());
    chooser.addOption("Place Cone High Balance Old", new PlaceHighConeBalanceOld());
    chooser.addOption("Place Cone High Back", new PlaceHighConeBackDrive());   
    chooser.addOption("Place Cube High", new PlaceHighCube());
    chooser.addOption("Place Cube High Balance", new PlaceHighCubeAndBalance());
    chooser.addOption("Place Cube High Back", new PlaceHighCubeBackDrive());
    chooser.addOption("Place Cone Mid", new PlaceMidCone());
    chooser.addOption("Place Cone Mid Balance", new PlaceMidConeAndBalance());
    chooser.addOption("Place Cone Mid Back", new PlaceMidConeBackDrive());
    chooser.addOption("Place Cube Mid", new PlaceMidCube());
    chooser.addOption("Place Cube Mid Balance", new PlaceMidCubeAndBalance());
    chooser.addOption("Place Cube Mid Back ", new PlaceMidCubeBackDrive());

    // Motion Profiles
    //try { chooser.addOption("MP Test ", new MPTest()); } catch (Exception e) { }
    try { chooser.addOption("MP Forward ", new ForwardTest()); } catch (Exception e) { }
    //try { chooser.addOption("MP Forward Back ", new ForwardBackTest()); } catch (Exception e) { }
    //try { chooser.addOption("MP Curve Right ", new CurveRightTest()); } catch (Exception e) { }
    //try { chooser.addOption("MP 1m Square ", new Square1M()); } catch (Exception e) { }
    //try { chooser.addOption("Two Cone Blue", new TwoConeBlue()); } catch (Exception e) { }
    //try { chooser.addOption("Two Cone Blue Test", new TwoConeBlueTest()); } catch (Exception e) { }
    //try { chooser.addOption("Two Cone Red", new TwoConeRed()); } catch (Exception e) { }
    //try { chooser.addOption("Two Cone Red Test", new TwoConeRedTest()); } catch (Exception e) { }
    try { chooser.addOption("Diagonal", new Diagonal()); } catch (Exception e) { }
    try { chooser.addOption("Diagonal Twist", new DiagonalTwist()); } catch (Exception e) { }
    try { chooser.addOption("L", new L()); } catch (Exception e) { }

		SmartDashboard.putData("Auto Chooser", chooser);
	}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return this.chooser.getSelected();
  }
}
