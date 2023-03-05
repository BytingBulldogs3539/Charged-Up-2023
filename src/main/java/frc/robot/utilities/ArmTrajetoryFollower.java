package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.ElevatorConstants;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.swervedrivespecialties.swervelib.control.Trajectory2;
import com.swervedrivespecialties.swervelib.control.Path.State;

import java.awt.geom.Point2D;

public class ArmTrajetoryFollower extends CommandBase {
	private final Timer m_timer = new Timer();

	State lastState = null;

	private Trajectory2 m_trajectory;
	private final Supplier<Point2D.Double> endPointSupplier;
	private final ArmTrajectoryGenerator generator;
	private final Supplier<ArmPosition> m_pose;
	private final Supplier<Point2D.Double> xyPose;
	private final Consumer<Double> setExtenionSpeed;
	private final Consumer<Double> setRotationSpeed;
	private final PIDController m_rController;
	private final PIDController m_eController;
	private final double xKf;
	private double minArmLength = 0.0;
	private double lastTime = 0.0;
	private double maxArmLength = 100.0;
	private Rotation2d minArmRotation = Rotation2d.fromDegrees(0);
	private Rotation2d maxArmRotation = Rotation2d.fromDegrees(0);
	private Point2D.Double lastEndpoint = new Point2D.Double(0, 0);

	public ArmTrajetoryFollower(Supplier<Point2D.Double> endPoint, ArmTrajectoryGenerator generator,
			Supplier<ArmPosition> pose, Supplier<Point2D.Double> xyPose,
			Consumer<Double> setExtenionSpeed,
			Consumer<Double> setRotationSpeed, PIDController rController, double rKf,
			PIDController eController, double minArmLength, double maxArmLength, Rotation2d minArmRotation,
			Rotation2d maxArmRotation, Subsystem... requirements) {
		this.endPointSupplier = (Supplier<Point2D.Double>) ErrorMessages.requireNonNullParam(endPoint, "endPoint",
				"SwerveControllerCommand");
		this.generator = (ArmTrajectoryGenerator) ErrorMessages.requireNonNullParam(generator, "generator",
				"SwerveControllerCommand");
		this.m_pose = (Supplier<ArmPosition>) ErrorMessages.requireNonNullParam(pose, "pose",
				"SwerveControllerCommand");
		this.xyPose = (Supplier<Point2D.Double>) ErrorMessages.requireNonNullParam(xyPose, "xyPose",
				"SwerveControllerCommand");
		this.m_rController = (PIDController) ErrorMessages.requireNonNullParam(rController, "xController",
				"SwerveControllerCommand");
		this.xKf = (double) ErrorMessages.requireNonNullParam(rKf, "xKf",
				"SwerveControllerCommand");
		this.m_eController = (PIDController) ErrorMessages.requireNonNullParam(eController, "xController",
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
		m_trajectory = generator.generateTrajectories(xyPose.get(), endPointSupplier.get());
		lastState = m_trajectory.calculate(0).getPathState();

	}

	public void execute() {
		// System.out.println(endPointSupplier.get().x+" " + endPointSupplier.get().y);
		if (endPointSupplier.get().x != lastEndpoint.x || endPointSupplier.get().y != lastEndpoint.y) {
			// System.out.println(xyPose.get().x+" " +xyPose.get().y);
			// System.out.println(endPointSupplier.get().x+" " + endPointSupplier.get().y);
			double endPointx = Math.floor(endPointSupplier.get().x * 100) / 100.0;
			double endPointy = Math.floor(endPointSupplier.get().y * 100) / 100.0;
			double startPointx = Math.floor(xyPose.get().x * 100) / 100.0;
			double startPointy = Math.floor(xyPose.get().y * 100) / 100.0;
			m_trajectory = generator.generateTrajectories(new Point2D.Double(startPointx, startPointy),
					new Point2D.Double(endPointx, endPointy));

			// System.out.println(m_trajectory.getDuration());
			this.m_timer.stop();
			this.m_timer.reset();
			this.m_timer.start();
			lastState = m_trajectory.calculate(0).getPathState();
			// System.out.println("yolo");
			// System.out.println(endPointSupplier.get().x +" : "+endPointSupplier.get().y);
		}

		double curTime = this.m_timer.get();
		SmartDashboard.putNumber("CurTime", curTime);

		State s = this.m_trajectory.calculate(curTime).getPathState();

		ArmPosition lastP = ArmTrajectoryGenerator.xYToPolar(lastState.getPosition().x, lastState.getPosition().y);

		ArmPosition p = ArmTrajectoryGenerator.xYToPolar(s.getPosition().x, s.getPosition().y);

		if (p.getRotation().getDegrees() < -95) {
			p = new ArmPosition(Rotation2d.fromDegrees(p.getRotation().getDegrees() + 360), p.getExtension());
		}

		if (p.getExtension() < ElevatorConstants.ElevatorMinExtension) {
			p = new ArmPosition(p.getRotation(), minArmLength);
		}
		
		if (lastP.getRotation().getDegrees() < -95) {
			lastP = new ArmPosition(Rotation2d.fromDegrees(p.getRotation().getDegrees() + 360), lastP.getExtension());
		}

		if (lastP.getExtension() < ElevatorConstants.ElevatorMinExtension) {
			lastP = new ArmPosition(lastP.getRotation(), minArmLength);
		}

		//double rotationVelocity = (((p.getRotation().getDegrees() - lastP.getRotation().getDegrees())
		//		/ (curTime - lastTime)) / 360.0) * 60.0;// rotations per min.

		//double velocityFeedForward = (rotationVelocity/ElevatorConstants.ElevatorRotationGearRatio)*ElevatorConstants.ElevatorRotationVelocityFeedforwardRatio;


		// if (p.getExtension() < minArmLength) {
		// p = new ArmPosition(p.getRotation(), minArmLength);
		// }
		// if (p.getExtension() > maxArmLength) {
		// p = new ArmPosition(p.getRotation(), maxArmLength);
		// }
		// if (p.getRotation().getDegrees() > maxArmRotation.getDegrees()) {
		// p = new ArmPosition(maxArmRotation, p.getExtension());
		// }
		// if (p.getRotation().getDegrees() < minArmRotation.getDegrees()) {
		// p = new ArmPosition(minArmRotation, p.getExtension());
		// }

		// TODO: add limiting box.

		SmartDashboard.putNumber("Expected Arm X", s.getPosition().x);
		SmartDashboard.putNumber("Expected Arm Y", s.getPosition().y);

		SmartDashboard.putNumber("Expected Extension", p.getExtension());
		SmartDashboard.putNumber("Expected Rotation", p.getRotation().getDegrees());

		SmartDashboard.putNumber("Real Extension", this.m_pose.get().getExtension());
		SmartDashboard.putNumber("Real Rotation", this.m_pose.get().getRotation().getDegrees());

		SmartDashboard.putNumber("Real Arm x", xyPose.get().getX());
		SmartDashboard.putNumber("Real Arm y", xyPose.get().getY());

		double targetE = this.m_eController.calculate(this.m_pose.get().getExtension(), p.getExtension());
		double targetR = this.m_rController
				.calculate(this.m_pose.get().getRotation().getDegrees(), p.getRotation().getDegrees())
				+ (xKf * s.getPosition().x);//+velocityFeedForward;

		setExtenionSpeed.accept(targetE);
		setRotationSpeed.accept(targetR);

		lastEndpoint = endPointSupplier.get();
		lastState = s;
		lastTime = curTime;
	}

	public void end(boolean interrupted) {
		this.m_timer.stop();
		setRotationSpeed.accept(0.0);
		setExtenionSpeed.accept(0.0);
	}

	public boolean isFinished() {
		return false;
	}
}