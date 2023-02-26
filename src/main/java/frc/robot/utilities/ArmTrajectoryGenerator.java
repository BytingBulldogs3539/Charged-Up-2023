// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.swervedrivespecialties.swervelib.control.CentripetalAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.Path;
import com.swervedrivespecialties.swervelib.control.SimplePathBuilder;
import com.swervedrivespecialties.swervelib.control.Trajectory;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;
import com.swervedrivespecialties.swervelib.control.Trajectory.State;
import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ArmTrajectoryGenerator {
    double armLength;
    double maximumVelocity;
    double maximumAcceleration;

    public ArmTrajectoryGenerator(double maximumVelocity, double maximumAcceleration, double armLength) {
        this.maximumAcceleration = maximumAcceleration;
        this.maximumVelocity = maximumVelocity;
        this.armLength = armLength;
    }

    public Trajectory generateTrajectories(Point2D.Double startPoint, Point2D.Double endPoint) {
        ArmPosition startPointPolar = xYToPolar(startPoint);
        ArmPosition retractPointPolar = new ArmPosition(startPointPolar.getRotation(), armLength);
        Point2D retractPoint2d = retractPointPolar.polarToXY();
        Vector2 retractPoint = new Vector2(retractPoint2d.getX(), retractPoint2d.getY());

        boolean useMidPoint = false;
        if (startPoint.x > 0 && endPoint.x < 0) {
            useMidPoint = true;
        } else if (startPoint.x < 0 && endPoint.x > 0) {
            useMidPoint = true;
        }

        ArmPosition endPointPolar = xYToPolar(endPoint);
        ArmPosition extensionPointPolar = new ArmPosition(endPointPolar.getRotation(), armLength);
        Point2D extensionPoint2d = extensionPointPolar.polarToXY();
        Vector2 extensionPoint = new Vector2(extensionPoint2d.getX(), extensionPoint2d.getY());
        boolean sameQuad = sameQuadrant(startPoint, endPoint);

        SimplePathBuilder p = new SimplePathBuilder(new Vector2(startPoint.x, startPoint.y), Rotation2.ZERO);
        if (sameQuad) {
            p.lineTo(new Vector2(endPoint.x, endPoint.y));
        } else {
            p.lineTo(retractPoint);
            if (useMidPoint) {
                p.arcTo(new Vector2(0, armLength), new Vector2(0, 0));
                p.arcTo(extensionPoint, new Vector2(0, 0));
            }
            else
            {
                p.arcTo(extensionPoint, new Vector2(0, 0));
            }
            p.lineTo(new Vector2(endPoint.x, endPoint.y));
        }

        Path path = p.build();

        TrajectoryConstraint[] constraints = {
                (TrajectoryConstraint) new MaxAccelerationConstraint(maximumAcceleration),
                (TrajectoryConstraint) new MaxVelocityConstraint(maximumVelocity) };
        return new Trajectory(path, constraints, 3);
    }

    public static boolean sameQuadrant(Point2D p1, Point2D p2) {
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        if ((x1 > 0 && x2 > 0) && (y1 > 0 && y2 > 0)) {
            // both points are in quadrant 1
            return true;
        } else if ((x1 < 0 && x2 < 0) && (y1 > 0 && y2 > 0)) {
            // both points are in quadrant 2
            return true;
        } else if ((x1 < 0 && x2 < 0) && (y1 < 0 && y2 < 0)) {
            // both points are in quadrant 3
            return true;
        } else if ((x1 > 0 && x2 > 0) && (y1 < 0 && y2 < 0)) {
            // both points are in quadrant 4
            return true;
        } else {
            // the points are not in the same quadrant
            return false;
        }
    }

    public static Point2D.Double polarToXY(ArmPosition p) {
        return polarToXY(p.getRotation().getRadians(), p.getExtension());
    }

    public static ArmPosition xYToPolar(Point2D p) {
        return xYToPolar(p.getX(), p.getY());
    }

    public static Point2D.Double polarToXY(double angle, double length) {

        double x = Math.cos(angle) * length;
        double y = Math.sin(angle) * length;
        return new Point2D.Double(x, y);
    };

    public static ArmPosition xYToPolar(double x, double y) {

        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(y, x));
        double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        return new ArmPosition(angle, length);

    };

    public static void main(String[] args) {
        long startTime = System.nanoTime();

        ArmTrajectoryGenerator h = new ArmTrajectoryGenerator(120, 140, 78);
        int armLength = 78;
        Trajectory traj2 = h.generateTrajectories(new Point2D.Double(78, 78), new Point2D.Double(150, 0));

        ArrayList<ArmPosition> points = new ArrayList<ArmPosition>();

        for (double i = 0; i < traj2.getDuration(); i += 0.02) {

            State s = traj2.calculate(i);
            ArmPosition p = xYToPolar(s.getPathState().getPosition().x, s.getPathState().getPosition().y);
            if (p.getExtension() < armLength) {
                p = new ArmPosition(p.getRotation(), armLength);
            }
            points.add(p);
        }

        long endTime = System.nanoTime();
        System.out.println(TimeUnit.NANOSECONDS.toMillis(endTime - startTime));
        System.out.println(traj2.getDuration());

        for (ArmPosition point : points) {
            System.out.printf("(%.3f,%.3f)\n", ArmTrajectoryGenerator.polarToXY(point).getX(),ArmTrajectoryGenerator.polarToXY(point).getY());

        }

    }
}
