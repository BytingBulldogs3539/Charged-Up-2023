// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.swervedrivespecialties.swervelib.control.CentripetalAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.Path;
import com.swervedrivespecialties.swervelib.control.PathSegment;
import com.swervedrivespecialties.swervelib.control.SimplePathBuilder;
import com.swervedrivespecialties.swervelib.control.Trajectory;
import com.swervedrivespecialties.swervelib.control.Trajectory2;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;
import com.swervedrivespecialties.swervelib.control.Path.State;
import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;
import com.swervedrivespecialties.swervelib.util.InterpolatingDouble;
import com.swervedrivespecialties.swervelib.util.InterpolatingTreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.ElevatorConstants;

/** Add your docs here. */
public class ArmTrajectoryGenerator {
    double armLength;
    double maximumVelocity;
    double maximumAcceleration;
    double maximumArcVelcity;
    double maximumArcAcceleration;

    public ArmTrajectoryGenerator(double maximumVelocity, double maximumAcceleration, double maximumArcVelocity, double maximumArcAcceleration, double armLength) {
        this.maximumAcceleration = maximumAcceleration;
        this.maximumVelocity = maximumVelocity;
        this.armLength = armLength;
        this.maximumArcAcceleration = maximumArcAcceleration;
        this.maximumArcVelcity = maximumArcVelocity;
    }

    public MultiTrajectory generateTrajectories(Point2D.Double startPoint, Point2D.Double endPoint) {
        ArmPosition startPointPolar = xYToPolar(startPoint);
        ArmPosition retractPointPolar = new ArmPosition(startPointPolar.getRotation(), armLength);
        Point2D retractPoint2d = retractPointPolar.polarToXY();
        Vector2 retractPoint = new Vector2(retractPoint2d.getX(), retractPoint2d.getY());

        boolean clockwise = false;
        if (startPoint.x < 0 && endPoint.x > 0) {
            clockwise = true;
        }
        else if((startPoint.x < 0 && endPoint.x < 0) &&(startPoint.y > endPoint.y))
        {
            clockwise = false;
        }
        else if((startPoint.x < 0 && endPoint.x < 0) &&(startPoint.y < endPoint.y))
        {
            clockwise = true;
        }
        else if((startPoint.x > 0 && endPoint.x > 0) &&(startPoint.y < endPoint.y))
        {
            clockwise = false;
        }
        else if((startPoint.x > 0 && endPoint.x > 0) &&(startPoint.y > endPoint.y))
        {
            clockwise = true;
        }
        else if((startPoint.x == 0 && endPoint.x > 0))
        {
            clockwise = true;
        }
        else if((startPoint.x == 0 && endPoint.x < 0))
        {
            clockwise = true;
        }




        ArmPosition endPointPolar = xYToPolar(endPoint);
        ArmPosition extensionPointPolar = new ArmPosition(endPointPolar.getRotation(), armLength+.1);
        Point2D extensionPoint2d = extensionPointPolar.polarToXY();
        Vector2 extensionPoint = new Vector2(extensionPoint2d.getX(), extensionPoint2d.getY());
        boolean sameQuad = sameQuadrant(startPoint, endPoint);


        TrajectoryConstraint[] constraints = {
            (TrajectoryConstraint) new MaxAccelerationConstraint(maximumAcceleration),
            (TrajectoryConstraint) new MaxVelocityConstraint(maximumVelocity),
            (TrajectoryConstraint) new CentripetalAccelerationConstraint(100) };

    TrajectoryConstraint[]  RotationConstraints = {
                (TrajectoryConstraint) new MaxAccelerationConstraint(maximumArcAcceleration),
                (TrajectoryConstraint) new MaxVelocityConstraint(maximumArcVelcity)};

        SimplePathBuilder p1 = new SimplePathBuilder(new Vector2(startPoint.x, startPoint.y), Rotation2.ZERO);
        Path path;
        if (sameQuad) {
            p1.lineTo(new Vector2(endPoint.x, endPoint.y));
            path = p1.build();
            return new MultiTrajectory(new Trajectory2(path, constraints, 3));
        } else {
            p1.lineTo(retractPoint);

            SimplePathBuilder p2 = new SimplePathBuilder(retractPoint, Rotation2.ZERO);

            p2.arcTo(extensionPoint, new Vector2(0, 0), clockwise);

            SimplePathBuilder p3 = new SimplePathBuilder(extensionPoint, Rotation2.ZERO);
            p3.lineTo(new Vector2(endPoint.x, endPoint.y));

            Path path1 = p1.build();
            Path path2 = p2.build();
            Path path3 = p3.build();
            return new MultiTrajectory(new Trajectory2(path1, constraints, 3),new Trajectory2(path2, RotationConstraints, 3),new Trajectory2(path3, constraints, 3));
            
        }
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
        // trajectoryHandler = new
        // ArmTrajectoryGenerator(ElevatorConstants.maxArmVelocity,
        // ElevatorConstants.maxArmAcceleration,
        // ElevatorConstants.ElevatorMinExtension);
        ArmTrajectoryGenerator h = new ArmTrajectoryGenerator(200, 200, 500, 500, 78.74);
        MultiTrajectory traj2 = h.generateTrajectories(
                new Point2D.Double(ElevatorConstants.frontConeIntakeX, ElevatorConstants.frontConeIntakeY),
                new Point2D.Double(ElevatorConstants.backCubeIntakeX, ElevatorConstants.backConeIntakeY));

        ArrayList<ArmPosition> points = new ArrayList<ArmPosition>();

        for (double i = 0; i < traj2.getDuration(); i += 0.02) {

            State s = traj2.calculate(i).getPathState();

            ArmPosition p = ArmTrajectoryGenerator.xYToPolar(s.getPosition().x, s.getPosition().y);

            if (p.getRotation().getDegrees() < -95) {
                p = new ArmPosition(Rotation2d.fromDegrees(p.getRotation().getDegrees() + 360), p.getExtension());
            }
            // ArmPosition p = xYToPolar(s.getPathState().getPosition().x,
            // s.getPathState().getPosition().y);

            points.add(p);
        }

        for (ArmPosition point : points) {
            System.out.printf("(%.3f,%.3f)\n", ArmTrajectoryGenerator.polarToXY(point).getX(),
                    ArmTrajectoryGenerator.polarToXY(point).getY());

        }

    }
}
