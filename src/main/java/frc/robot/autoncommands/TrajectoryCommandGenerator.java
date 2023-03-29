package frc.robot.autoncommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.SwerveAutoBalanceController;
import frc.robot.utilities.SwerveController;
import com.swervedrivespecialties.swervelib.control.Path;
import com.swervedrivespecialties.swervelib.control.PidConstants;
import com.swervedrivespecialties.swervelib.control.PidController;
import com.swervedrivespecialties.swervelib.control.Trajectory;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;

public class TrajectoryCommandGenerator {
	public static Command getMotionCommand(Path path, TrajectoryConstraint[] constraints, DriveSubsystem driveSub) {
		Trajectory trajectory = new Trajectory(path, constraints, 0.2);
		SwerveController swerveControllerCommand = new SwerveController(trajectory, driveSub::getPose,
				new PidController(new PidConstants(
						DriveConstants.TranslationXkP,
						DriveConstants.TranslationXkI,
						DriveConstants.TranslationXkD
                        )), DriveConstants.TranslationXkF,

				new PidController(new PidConstants(
						DriveConstants.TranslationYkP,
						DriveConstants.TranslationYkI,
						DriveConstants.TranslationYkD

                )), DriveConstants.TranslationYkF,
                    
				new PidController(new PidConstants(
						DriveConstants.RotationkP,
						DriveConstants.RotationkI,
						DriveConstants.RotationkD)),driveSub,driveSub);

		return (Command) swerveControllerCommand.andThen(() -> driveSub.drive(new ChassisSpeeds(0, 0, 0)),
				new Subsystem[0]);
	}

	public static Command getAutoBalanceMotionCommand(Path path, TrajectoryConstraint[] constraints, DriveSubsystem driveSub) {
		Trajectory trajectory = new Trajectory(path, constraints, 0.2);
		SwerveAutoBalanceController swerveControllerCommand = new SwerveAutoBalanceController(trajectory, driveSub::getPose, driveSub::getPitch,
				new PidController(new PidConstants(
						DriveConstants.TranslationXkP,
						DriveConstants.TranslationXkI,
						DriveConstants.TranslationXkD
                        )), DriveConstants.TranslationXkF,

				new PidController(new PidConstants(
						DriveConstants.TranslationYkP,
						DriveConstants.TranslationYkI,
						DriveConstants.TranslationYkD

                )), DriveConstants.TranslationYkF,
                    
				new PidController(new PidConstants(
						DriveConstants.RotationkP,
						DriveConstants.RotationkI,
						DriveConstants.RotationkD)),driveSub,driveSub);

		return (Command) swerveControllerCommand.andThen(() -> driveSub.drive(new ChassisSpeeds(0, 0, 0)),
				new Subsystem[0]);
	}
}