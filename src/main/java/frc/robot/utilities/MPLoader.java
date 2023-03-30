// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.SplinePathBuilder;
import com.swervedrivespecialties.swervelib.control.SimplePathBuilder;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;
import com.swervedrivespecialties.swervelib.math.Rotation2;
import com.swervedrivespecialties.swervelib.math.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.RobotContainer;
import frc.robot.autoncommands.TrajectoryCommandGenerator;
import frc.robot.autoncommands.SetPoseCommand;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

public class MPLoader {
    // Fill a command sequence from .txt file
    // (recommended for testing only)
    public static Command[] getCommandSequence(String filename) {
        filename = "/home/lvuser/profiles/" + filename;
        System.out.println("\n\n\n\nfilename: " + filename + "\n\nlines:");

        ArrayList<ArrayList<String>> paths = new ArrayList<ArrayList<String>>();
        ArrayList<String> path = new ArrayList<String>();
        ArrayList<String> constraints = new ArrayList<String>();
        String line;

        try {
            BufferedReader reader = new BufferedReader(new FileReader(filename));
            while ((line = reader.readLine()) != null) {
                if (line.startsWith("@")) {
                    // End of path detected
                    paths.add(path);
                    path = new ArrayList<String>();
                }
                else if (paths.size() == constraints.size()) {
                    // Add path constraints
                    constraints.add(line);
                }
                else {
                    path.add(line);
                }
            }
        } catch (Exception e) { }
        
        double[][][] pathsParsed = new double[paths.size()][][];
        for (int i = 0; i < paths.size(); i++) {
            double[][] pathParsed = new double[paths.get(i).size()][];
            for (int j = 0; j < paths.get(i).size(); j++) {
                String points = paths.get(i).get(j);
                String[] pointData = points.split(" ");
                double[] parsedPoints = new double[5];
                for (int k = 0; k < 5; k++) {
                    parsedPoints[k] = Double.parseDouble(pointData[k]);
                }
                pathParsed[j] = parsedPoints;
            }
            pathsParsed[i] = pathParsed;
        }

        double[][] constraintsParsed = new double[paths.size()][];
        for (int i = 0; i < paths.size(); i++) {
            String[] constraintsList = constraints.get(i).split(" ");
            constraintsParsed[i] = new double[] {
                Double.parseDouble(constraintsList[0]),
                Double.parseDouble(constraintsList[1])
            };
        }

        return getCommandSequence(pathsParsed, constraintsParsed);
    }

    // Fill a command sequence from a pre-compiled or parsed double array
    // (recommended for comp)
    public static Command[] getCommandSequence(double[][][] paths, double[][] constraints) {
        System.out.println("received:");
        for (double[][] ds : paths) {
			for (double[] ds2 : ds) {
				for (double ds3 : ds2) {
					System.out.print(ds3 + " ");
				}
				System.out.println();
			}
		}
        System.out.println("\n\n\n");

        Command[] sequence = new Command[paths.length + 1];
        sequence[0] = new SetPoseCommand(
            new Pose2d(
                paths[0][0][0],
                paths[0][0][1],
                Rotation2d.fromDegrees(paths[0][0][4])
            )
        );
        int pathIdx = 0;
        for (int command = 1; command < sequence.length; command++) {
            SimplePathBuilder builder = new SimplePathBuilder(
                new Vector2(paths[pathIdx][0][0], paths[pathIdx][0][1]),
                Rotation2.fromDegrees(paths[pathIdx][0][4])
            );
            /*
            SplinePathBuilder builder = new SplinePathBuilder(
                // Initial location
                new Vector2(paths[pathIdx][0][0], paths[pathIdx][0][1]),
                // Initial path angle (pre-normalized)
                new Rotation2(paths[pathIdx][0][2], paths[pathIdx][0][3], false),
                // Initial robot heading (not normalized)
                Rotation2.fromDegrees(paths[pathIdx][0][4])
            ); */
            for (int point = 1; point < paths[pathIdx].length; point++) {
                /*
                builder.hermite(
                    // Add point location
                    new Vector2(paths[pathIdx][point][0], paths[pathIdx][point][1]),
                    // Add path angle at point (pre-normalized)
                    new Rotation2(paths[pathIdx][point][2], paths[pathIdx][point][3], false),
                    // Add robot heading at point (not normalized)
                    Rotation2.fromDegrees(paths[pathIdx][point][4])
                ); */
                builder.lineTo(
                    new Vector2(paths[pathIdx][point][0], paths[pathIdx][point][1]),
                    Rotation2.fromDegrees(paths[pathIdx][point][4])
                );
            }
            sequence[command] = TrajectoryCommandGenerator.getMotionCommand(
                builder.build(),
                getConstraints(constraints[pathIdx][0], constraints[pathIdx][1]),
                RobotContainer.driveSubsystem
            );
            pathIdx++;
        }
		return sequence;
	}

    public static TrajectoryConstraint[] getConstraints(double velocity, double acceleration) {
        return new TrajectoryConstraint[] {
            (TrajectoryConstraint) new MaxAccelerationConstraint(acceleration),
            (TrajectoryConstraint) new MaxVelocityConstraint(velocity)
        };
    }
}