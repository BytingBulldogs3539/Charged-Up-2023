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

/** Add your docs here. */
public class ArmTrajectoryHandler {
    TrajectoryConfig config;
    double armLength;
    Point2D.Double midPoint;

    public ArmTrajectoryHandler(double maximumVelocity, double maximumAcceleration, double armLength, Point2D.Double midPoint) {

        config = new TrajectoryConfig(maximumVelocity, maximumAcceleration);
        this.armLength = armLength;
        this.midPoint = midPoint;
    }

    public ArrayList<Point2D> generateTrajectories(Point2D.Double startPoint, Point2D.Double endPoint) {
        double differenceInX = endPoint.getX() - startPoint.getX();// -60
        double differenceInY = endPoint.getY() - startPoint.getY();// -60
        double startAngle = 0;
        double endAngle = 0;

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

        Pose2d startPose = new Pose2d(startPoint.getX(), startPoint.getY(), Rotation2d.fromDegrees(90));

        if (differenceInX < 0) {
            endAngle = 180;
        } else if (differenceInX > 0) {
            endAngle = 0;
        } else {
            endAngle = startAngle;
        }
        Pose2d endPose = new Pose2d(endPoint.getX(), endPoint.getY(), Rotation2d.fromDegrees(endAngle));

        if(startPose.getX()>0 && endPose.getX()<0)
        {
            startPose = new Pose2d(startPoint.getX(), startPoint.getY(), Rotation2d.fromDegrees(180));

        }
        if(startPose.getX()<0 && endPose.getX()>0)
        {
            startPose = new Pose2d(startPoint.getX(), startPoint.getY(), Rotation2d.fromDegrees(0));

        }


        List<Pose2d> list = new ArrayList<Pose2d>();
        list.add(startPose);
        if(startPose.getX()>0 && endPose.getX()<0)
        {
            list.add(new Pose2d(midPoint.getX(), midPoint.getY(), Rotation2d.fromDegrees(180)));
        }
        if(startPose.getX()<0 && endPose.getX()>0)
        {
            list.add(new Pose2d(midPoint.getX(), midPoint.getY(), Rotation2d.fromDegrees(0)));
        }
        list.add(endPose);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(list, this.config);



        ArrayList<Point2D> output = new ArrayList<Point2D>();
        //System.out.println(startAngle);
        //System.out.println(endAngle);
        
        for (double i = 0; i < traj.getTotalTimeSeconds(); i += 0.015) {
            State s = traj.sample(i);
            //System.out.println("(" + s.poseMeters.getX() + " ," + s.poseMeters.getY() + ")");
            Point2D p = xYToPolar(s.poseMeters.getX(), s.poseMeters.getY());

            if (p.getY() < armLength) {
                p = new Point2D.Double(p.getX(), armLength);

            }
            output.add(p);
        }
        return output;
    }

    public Point2D polarToXY(double angle, double length) {

        double x = Math.cos(angle) * length;
        double y = Math.sin(angle) * length;
        return new Point2D.Double(x, y);
    };

    public Point2D xYToPolar(double x, double y) {

        double angle = Math.toDegrees(Math.atan2(y, x));
        double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        if (angle < 0) {
            angle += 360;
        }

        return new Point2D.Double(angle, length);

    };

    public static void main(String[] args) {
        ArmTrajectoryHandler h = new ArmTrajectoryHandler(60, 60, 25, new Point2D.Double(0, 25));
        ArrayList<Point2D> points = h.generateTrajectories(new Point2D.Double(-50, 0), new Point2D.Double(20, -15));
        for (Point2D point : points) {
            System.out.println("("+h.polarToXY(Math.toRadians(point.getX()), point.getY()).getX()+","+h.polarToXY(Math.toRadians(point.getX()), point.getY()).getY()+")");

        }
    }
}
