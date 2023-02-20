package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;
import com.swervedrivespecialties.swervelib.control.Path;
import com.swervedrivespecialties.swervelib.control.PidController;
import com.swervedrivespecialties.swervelib.control.Trajectory;

public class SwerveController extends CommandBase {
    private final Timer m_timer = new Timer();

    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final PidController m_xController;
    private final PidController m_yController;
    private final PidController m_thetaController;
    private final DriveSubsystem m_driveSub;
    private final double xKf;
    private final double yKf;
    private double lastTime = 0.0D;

    public SwerveController(Trajectory trajectory, Supplier<Pose2d> pose, PidController xController, double xKf,
            PidController yController, double yKf, PidController thetaController, DriveSubsystem driveSub,
            Subsystem... requirements) {
        this.m_trajectory = (Trajectory) ErrorMessages.requireNonNullParam(trajectory, "trajectory",
                "SwerveControllerCommand");
        this.m_pose = (Supplier<Pose2d>) ErrorMessages.requireNonNullParam(pose, "pose",
                "SwerveControllerCommand");

        this.m_xController = (PidController) ErrorMessages.requireNonNullParam(xController, "xController",
                "SwerveControllerCommand");
        this.xKf = (double) ErrorMessages.requireNonNullParam(xKf, "xKf",
                "SwerveControllerCommand");
        this.m_yController = (PidController) ErrorMessages.requireNonNullParam(yController, "xController",
                "SwerveControllerCommand");
        this.yKf = (double) ErrorMessages.requireNonNullParam(yKf, "yKf",
                "SwerveControllerCommand");
        this.m_thetaController = (PidController) ErrorMessages.requireNonNullParam(thetaController,
                "thetaController",
                "SwerveControllerCommand");
        this.m_driveSub = (DriveSubsystem) ErrorMessages.requireNonNullParam(driveSub, "driveSub",
                "SwerveControllerCommand");
        addRequirements(requirements);
    }

    public void initialize() {
        this.m_timer.reset();
        this.m_timer.start();

        this.m_thetaController.setInputRange(0.0D, 2 * Math.PI);
        this.m_thetaController.setOutputRange(-1.0D, 1.0D);
        this.m_thetaController.setContinuous(true);
    }

    public void execute() {
        double curTime = this.m_timer.get();

        double dt = curTime - this.lastTime;

        Trajectory.State lastDesiredState = this.m_trajectory.calculate(this.lastTime);
        Path.State lastDesiredPose = lastDesiredState.getPathState();

        Trajectory.State desiredState = this.m_trajectory.calculate(curTime);
        Path.State desiredPose = desiredState.getPathState();
        double desiredVelocity = desiredState.getVelocity();

        SmartDashboard.putNumber("Expected X", (desiredPose.getPosition()).x);
        SmartDashboard.putNumber("Expected Y", (desiredPose.getPosition()).y);
        SmartDashboard.putNumber("Expected Angle", desiredPose.getRotation().inverse().toDegrees());

        SmartDashboard.putNumber("Real X", this.m_pose.get().getTranslation().getX());
        SmartDashboard.putNumber("Real Y", this.m_pose.get().getTranslation().getY());
        SmartDashboard.putNumber("Real Angle", this.m_pose.get().getRotation().getDegrees());

        this.m_xController.setSetpoint((desiredPose.getPosition()).x);
        double angle = Math.atan2(((desiredPose.getPosition()).y - (lastDesiredPose.getPosition()).y),
                ((desiredPose.getPosition()).x - (lastDesiredPose.getPosition()).x));

        // Accounts for when the angle should be 180 degrees but is actually 0
        // TODO: FIXED WITH ATAN2?
        /*
         * if (((desiredPose.getPosition()).x - (lastDesiredPose.getPosition()).x) < 0.0
         * && desiredPose.getPosition().y - (lastDesiredPose.getPosition()).y == 0) {
         * angle = Math.PI;
         * }
         */

        double xvel = Math.cos(angle) * desiredVelocity;

        SmartDashboard.putNumber("Speed/Vel x", xvel);

        double targetXVel = this.m_xController.calculate(((Pose2d) this.m_pose.get()).getTranslation().getX(),
                dt)
                + xvel * xKf;

        this.m_yController.setSetpoint((desiredPose.getPosition()).y);
        double yvel = Math.sin(angle) * desiredVelocity;

        SmartDashboard.putNumber("Speed/Vel y", yvel);

        double targetYVel = this.m_yController.calculate(((Pose2d) this.m_pose.get()).getTranslation().getY(),
                dt)
                + yvel * yKf;


        this.m_thetaController.setSetpoint(desiredPose.getRotation().inverse().toRadians());

        double targetAngularVel = this.m_thetaController
                .calculate(((Pose2d) this.m_pose.get()).getRotation().getRadians(), dt);

        m_driveSub.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        targetXVel,
                        targetYVel,
                        targetAngularVel * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        m_driveSub.getGyroscopeRotation()));

        this.lastTime = this.m_timer.get();
    }

    public void end(boolean interrupted) {
        this.m_timer.stop();
    }

    public boolean isFinished() {
        return this.m_timer.advanceIfElapsed(this.m_trajectory.getDuration());
    }
}