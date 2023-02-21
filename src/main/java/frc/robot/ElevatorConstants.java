// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class ElevatorConstants extends BBConstants {

	public ElevatorConstants() {
		super("/home/lvuser/ElevatorConstants.ini", false);
	}

	public static double maxArmVelocity = 0.0;
	public static double maxArmAcceleration = 0.0;

	public static double ElevatorConversionRatio = 0.0;
	public static double ElevatorRotationMagnetOffset = 0.0;

	public static double ElevatorKp = 0.0;

	public static double ElevatorKi = 0.0;

	public static double ElevatorKd = 0.0;

	public static double ElevatorRotationKp = 0.0;

	public static double ElevatorRotationKi = 0.0;

	public static double ElevatorRotationKd = 0.0;

	public static double ElevatorMinExtension = 0.0;
	public static double ElevatorMaxExtension = 0.0;

	public static double WristRotationMagnetOffset = 0.0;

	public static double ElevatorRotationFeedforwardRatio = 0.0;

	public static int elevatorSoftMax = 0;
	public static int elevatorSoftMin = 0;

	public static int elevatorRotationSoftMax = 0;
	public static int elevatorRotationSoftMin = 0;

	// Cubes
	public static double frontCubeHighX = 0;
	public static double frontCubeHighY = 0;

	public static double frontCubeMidX = 0;
	public static double frontCubeMidY = 0;

	public static double frontCubeLowX = 0;
	public static double frontCubeLowY = 0;

	public static double frontCubeIntakeX = 0;
	public static double frontCubeIntakeY = 0;

	public static double backCubeHighX = 0;
	public static double backCubeHighY = 0;

	public static double backCubeMidX = 0;
	public static double backCubeMidY = 0;

	public static double backCubeLowX = 0;
	public static double backCubeLowY = 0;

	public static double backCubeIntakeX = 0;
	public static double backCubeIntakeY = 0;

	// Cones
	public static double frontConeHighX = 0;
	public static double frontConeHighY = 0;

	public static double frontConeMidX = 0;
	public static double frontConeMidY = 0;

	public static double frontConeLowX = 0;
	public static double frontConeLowY = 0;

	public static double frontConeIntakeX = 0;
	public static double frontConeIntakeY = 0;

	public static double backConeHighX = 0;
	public static double backConeHighY = 0;

	public static double backConeMidX = 0;
	public static double backConeMidY = 0;

	public static double backConeLowX = 0;
	public static double backConeLowY = 0;

	public static double backConeIntakeX = 0;
	public static double backConeIntakeY = 0;

}
