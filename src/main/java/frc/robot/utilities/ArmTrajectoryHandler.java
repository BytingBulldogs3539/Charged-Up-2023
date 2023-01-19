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

/** Add your docs here. */
public class ArmTrajectoryHandler {
    TrajectoryConfig config;
    public ArmTrajectoryHandler(double maximumVelocity, double maximumAcceleration){
        
            config = new TrajectoryConfig(maximumVelocity, maximumAcceleration);
    }
    public void generateTrajectories(Point2D startPoint, Point2D endPoint){
       double differenceInX = endPoint.getX() - startPoint.getX();
       double differenceInY = endPoint.getY() - startPoint.getY();
       double startAngle = Math.atan(differenceInY/differenceInX);
       Pose2d startPose = new Pose2d(startPoint.getX(), startPoint.getY(), Rotation2d.fromRadians(startAngle));
       double endAngle = 0;
        if(differenceInX < 0 ){
            endAngle = 180;
        }
        else if(differenceInX > 0 ){
            endAngle = 0;
        }
        else{
            endAngle = startAngle;
        }
        Pose2d endPose = new Pose2d(endPoint.getX(), endPoint.getY(), Rotation2d.fromRadians(endAngle));
        List<Pose2d> list = new ArrayList<Pose2d>();
        list.add(startPose);
        list.add(endPose);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(list, this.config);
    }

}

