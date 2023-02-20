package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.swervedrivespecialties.swervelib.control.PidController;
import edu.wpi.first.math.trajectory.Trajectory.State;
import java.awt.geom.Point2D;

public class ArmTrajetoryFollower extends CommandBase {
	private final Timer m_timer = new Timer();

	private Trajectory m_trajectory;
	private final Point2D.Double endPoint;
	private final ArmTrajectoryGenerator generator;
	private final Supplier<ArmPosition> m_pose;
	private final Supplier<Point2D.Double> xyPose;
	private final Consumer<Double> setExtenionSpeed;
	private final Consumer<Double> setRotationSpeed;
	private final PidController m_rController;
	private final PidController m_eController;
	private final double xKf;
	private double lastTime = 0.0D;
	private double minArmLength = 0.0;
	private double maxArmLength = 100.0;
	private Rotation2d minArmRotation = Rotation2d.fromDegrees(0);
	private Rotation2d maxArmRotation = Rotation2d.fromDegrees(0);

	public ArmTrajetoryFollower(Point2D.Double endPoint,ArmTrajectoryGenerator generator,Supplier<ArmPosition> pose, Supplier<Point2D.Double> xyPose,
			Consumer<Double> setExtenionSpeed,
			Consumer<Double> setRotationSpeed, PidController rController, double rKf,
			PidController eController, double minArmLength, double maxArmLength, Rotation2d minArmRotation, Rotation2d maxArmRotation, Subsystem... requirements) {
		this.endPoint = (Point2D.Double) ErrorMessages.requireNonNullParam(endPoint, "endPoint",
		"SwerveControllerCommand");
		this.generator = (ArmTrajectoryGenerator) ErrorMessages.requireNonNullParam(generator, "generator",
		"SwerveControllerCommand");
		this.m_pose = (Supplier<ArmPosition>) ErrorMessages.requireNonNullParam(pose, "pose",
				"SwerveControllerCommand");
		this.xyPose = (Supplier<Point2D.Double>) ErrorMessages.requireNonNullParam(xyPose, "xyPose",
				"SwerveControllerCommand");
		this.m_rController = (PidController) ErrorMessages.requireNonNullParam(rController, "xController",
				"SwerveControllerCommand");
		this.xKf = (double) ErrorMessages.requireNonNullParam(rKf, "xKf",
				"SwerveControllerCommand");
		this.m_eController = (PidController) ErrorMessages.requireNonNullParam(eController, "xController",
				"SwerveControllerCommand");
		this.setExtenionSpeed = (Consumer<Double>) ErrorMessages.requireNonNullParam(setExtenionSpeed,
				"setExtensionSpeed",
				"SwerveControllerCommand");
		this.setRotationSpeed = (Consumer<Double>) ErrorMessages.requireNonNullParam(setRotationSpeed,
				"setRotationSpeed",
				"SwerveControllerCommand");

		this.minArmLength = minArmLength;
		this.maxArmLength = maxArmLength;
		this.minArmRotation = minArmRotation;
		this.maxArmRotation = maxArmRotation;

		addRequirements(requirements);
	}

	public void initialize() {
		this.m_timer.reset();
		this.m_timer.start();

		m_trajectory = generator.generateTrajectories(xyPose.get(), endPoint);
	}

	public void execute() {
		double curTime = this.m_timer.get();

		double dt = curTime - this.lastTime;

		State s = this.m_trajectory.sample(curTime);
		ArmPosition p = ArmTrajectoryGenerator.xYToPolar(s.poseMeters.getX(), s.poseMeters.getY());

		if (p.getExtension() < minArmLength) {
			p = new ArmPosition(p.getRotation(), minArmLength);
		}
		if (p.getExtension() > maxArmLength) {
			p = new ArmPosition(p.getRotation(), maxArmLength);
		}
		if (p.getRotation().getDegrees() > maxArmRotation.getDegrees()) {
			p = new ArmPosition(maxArmRotation, p.getExtension());
		}
		if (p.getRotation().getDegrees() < minArmRotation.getDegrees()) {
			p = new ArmPosition(minArmRotation, p.getExtension());
		}

		//TODO: add limiting box.

		SmartDashboard.putNumber("Expected Extension", p.getExtension());
		SmartDashboard.putNumber("Expected Rotation", p.getRotation().getDegrees());

		SmartDashboard.putNumber("Real Extension", this.m_pose.get().getExtension());
		SmartDashboard.putNumber("Real Rotation", this.m_pose.get().getRotation().getDegrees());

		this.m_rController.setSetpoint(p.getRotation().getDegrees());

		this.m_eController.setSetpoint(p.getExtension());

		double targetE = this.m_eController.calculate(this.m_pose.get().getExtension(), dt);
		double targetR = this.m_rController
				.calculate(this.m_pose.get().getRotation().getDegrees() + xKf * xyPose.get().getX(), dt);

		setExtenionSpeed.accept(targetE);
		setRotationSpeed.accept(targetR);

		this.lastTime = this.m_timer.get();
	}

	public void end(boolean interrupted) {
		this.m_timer.stop();
	}

	public boolean isFinished() {
		return false;
	}
}