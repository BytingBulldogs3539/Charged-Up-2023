// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.SplinePathBuilder;
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
    public static Command[] getCommandSequence(Command[] sequence, String filename) throws java.io.IOException {
        filename = "/home/lvuser/profiles/" + filename;
        System.out.println("\n\n\n\nfilename: " + filename + "\n\nlines:");
        ArrayList<ArrayList<String>> paths = new ArrayList<ArrayList<String>>();
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            ArrayList<String> path = new ArrayList<String>();
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.startsWith("@")) {
                    paths.add(path);
                    path = new ArrayList<String>();
                }
                else {
                    path.add(line);
                }
            }
            if (path.size() > 0) {
                paths.add(path);
            }
        }

        double[][][] pathsParsed = new double[paths.size()][][];
        for (int i = 0; i < paths.size(); i++) {
            double[][] path = new double[paths.get(i).size()][];
            for (int j = 0; j < paths.get(i).size(); j++) {
                String points = paths.get(i).get(j);
                String[] pointData = points.split(" ");
                double[] parsedPoints = new double[5];
                for (int k = 0; k < 5; k++) {
                    parsedPoints[k] = Double.parseDouble(pointData[k]);
                }
                path[j] = parsedPoints;
            }
            pathsParsed[i] = path;
        }

        return getCommandSequence(sequence, pathsParsed);
    }

    // Fill a command sequence from a pre-compiled or parsed double array
    // (recommended for comp)
    public static Command[] getCommandSequence(Command[] sequence, double[][][] paths) {
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
        int pathIdx = 0;
        boolean posed = false;
        for (int command = 0; command < sequence.length; command++) {
            if (sequence[command] == null && !posed) {
                sequence[command] = new SetPoseCommand(
                    new Pose2d(
                        paths[0][0][0],
                        paths[0][0][1],
                        Rotation2d.fromDegrees(paths[0][0][4])
                    )
                );
                posed = true;
            }
            else if (sequence[command] == null) {
                SplinePathBuilder builder = new SplinePathBuilder(
                    // Initial location
                    new Vector2(paths[pathIdx][0][0], paths[pathIdx][0][1]),
                    // Initial path angle (pre-normalized)
                    new Rotation2(paths[pathIdx][0][2], paths[pathIdx][0][3], false),
                    // Initial robot heading (not normalized)
                    Rotation2.fromDegrees(paths[pathIdx][0][4])
                );
                for (int point = 1; point < paths[pathIdx].length; point++) {
                    builder.hermite(
                        // Add point location
                        new Vector2(paths[pathIdx][point][0], paths[pathIdx][point][1]),
                        // Add path angle at point (pre-normalized)
                        new Rotation2(paths[pathIdx][point][2], paths[pathIdx][point][3], false),
                        // Add robot heading at point (not normalized)
                        Rotation2.fromDegrees(paths[pathIdx][point][4])
                    );
                }
                sequence[command] = TrajectoryCommandGenerator.getMotionCommand(
                    builder.build(),
                    getConstraints(),
                    RobotContainer.driveSubsystem
                );
                pathIdx++;
            }
        }
		return sequence;
	}

    public static TrajectoryConstraint[] getConstraints() {
        TrajectoryConstraint[] constraints = {
            (TrajectoryConstraint) new MaxAccelerationConstraint(0.5),
            (TrajectoryConstraint) new MaxVelocityConstraint(1) };
        return constraints;
    }
}