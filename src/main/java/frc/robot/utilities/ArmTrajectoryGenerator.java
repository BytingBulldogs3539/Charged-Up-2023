// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;

/** Add your docs here. */
public class ArmTrajectoryGenerator {
    TrajectoryConfig config;
    double armLength;
    Point2D.Double midPoint;
    Point2D.Double fakeMidPoint;

    public ArmTrajectoryGenerator(double maximumVelocity, double maximumAcceleration, double armLength, Point2D.Double midPoint, Point2D.Double fakeMidPoint) {

        config = new TrajectoryConfig(maximumVelocity, maximumAcceleration);
        this.armLength = armLength;
        this.midPoint = midPoint;
        this.fakeMidPoint = fakeMidPoint;
    }

    public Trajectory generateTrajectories(Point2D.Double startPoint, Point2D.Double endPoint) {
        double differenceInX = endPoint.getX() - startPoint.getX();
        double differenceInY = endPoint.getY() - startPoint.getY();
        double startAngle = 0;

        if (differenceInX == 0 && differenceInY > 0) {
            startAngle = 90;
        } else if (differenceInX == 0 && differenceInY < 0) {
            startAngle = 270;
        } else {
            startAngle = Math.toDegrees(Math.atan2(differenceInY, differenceInX));

            if (startAngle < 0) {
                startAngle += 360;
            }
        }

        Pose2d startPose = new Pose2d(startPoint.getX(), startPoint.getY(), Rotation2d.fromRadians(Math.atan2(startPoint.getY(), startPoint.getX())+Math.PI));//Rotation2d.fromDegrees(90));

        Pose2d endPose = new Pose2d(endPoint.getX(), endPoint.getY(), Rotation2d.fromRadians(Math.atan2(endPoint.getY(), endPoint.getX())));//Rotation2d.fromDegrees(endAngle));

        if(startPose.getX()>0 && endPose.getX()<0)
        {
            endPose = new Pose2d(endPoint.getX(), endPoint.getY(), Rotation2d.fromRadians(Math.atan2(endPoint.getY() - midPoint.getY(), endPoint.getX() - midPoint.getX())));
        }
        if(startPose.getX()<0 && endPose.getX()>0)
        {
            endPose = new Pose2d(endPoint.getX(), endPoint.getY(), Rotation2d.fromRadians(Math.atan2(endPoint.getY() - midPoint.getY(), endPoint.getX() - midPoint.getX())));
        }

        if(sameQuadrant(startPoint, endPoint))
        {
            startPose = new Pose2d(startPoint.getX(), startPoint.getY(), Rotation2d.fromRadians(Math.atan2(endPoint.getY() - startPoint.getY(), endPoint.getX() - startPoint.getX())));
            endPose = new Pose2d(endPoint.getX(), endPoint.getY(), Rotation2d.fromRadians(Math.atan2(endPoint.getY() - startPoint.getY(), endPoint.getX() - startPoint.getX())));
        }


        List<Pose2d> list = new ArrayList<Pose2d>();
        list.add(startPose);
        if(startPose.getX()>0 && endPose.getX()<0)
        {
            list.add(new Pose2d(fakeMidPoint.getX(), fakeMidPoint.getY(), Rotation2d.fromDegrees(180)));
        }
        if(startPose.getX()<0 && endPose.getX()>0)
        {
            list.add(new Pose2d(fakeMidPoint.getX(), fakeMidPoint.getY(), Rotation2d.fromDegrees(0)));
        }
        list.add(endPose);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(list, this.config);

        ArrayList<ArmPosition> output = new ArrayList<ArmPosition>();
        //System.out.println(startAngle);
        //System.out.println(endAngle);
        
        for (double i = 0; i < traj.getTotalTimeSeconds(); i += 0.015) {
            State s = traj.sample(i);
            //System.out.println("(" + s.poseMeters.getX() + " ," + s.poseMeters.getY() + ")");
            ArmPosition p = xYToPolar(s.poseMeters.getX(), s.poseMeters.getY());

            if (p.getExtension() < armLength) {
                p = new ArmPosition(p.getRotation(), armLength);
            }
            output.add(p);
        }
        List<Pose2d> controlPoints = new ArrayList<Pose2d>();
        for(int i =0; i<output.size()-1; i++)
        {
            Point2D one = polarToXY(output.get(i));
            Point2D two = polarToXY(output.get(i+1));
            double angle = Math.atan2(two.getY()-one.getY(), two.getX()-one.getX());
            controlPoints.add(new Pose2d(one.getX(), one.getY(), Rotation2d.fromRadians(angle)));
        }
        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(controlPoints, this.config.addConstraint(new CentripetalAccelerationConstraint(1000)));

        return traj2;
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
    public static Point2D polarToXY(ArmPosition p)
    {
        return polarToXY(p.getRotation().getRadians(), p.getExtension());
    }
    public static ArmPosition xYToPolar(Point2D p)
    {
        return xYToPolar(p.getX(), p.getY());
    }
    public static Point2D polarToXY(double angle, double length) {

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
        double armLength = 31;
        ArmTrajectoryGenerator h = new ArmTrajectoryGenerator(60, 140, 31, new Point2D.Double(0, 31), new Point2D.Double(0, 31));
        
        Trajectory traj2 = h.generateTrajectories(new Point2D.Double(31,0), new Point2D.Double(0, 31));

        ArrayList<ArmPosition> points = new ArrayList<ArmPosition>();
        
        for (double i = 0; i < traj2.getTotalTimeSeconds(); i += 0.015) {
            State s = traj2.sample(i);
            ArmPosition p = xYToPolar(s.poseMeters.getX(), s.poseMeters.getY());
            if (p.getExtension() < armLength) {
                p = new ArmPosition(p.getRotation(), armLength);
            }
            points.add(p);
        }

        System.out.println(traj2.getTotalTimeSeconds());

        for (ArmPosition point : points) {
            System.out.println("("+ArmTrajectoryGenerator.polarToXY(point).getX()+","+ArmTrajectoryGenerator.polarToXY(point).getY()+")");
        }
    }
}
