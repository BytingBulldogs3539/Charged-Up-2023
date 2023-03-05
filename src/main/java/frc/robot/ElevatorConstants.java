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

	public static double maxArmVelocity = 150;
	public static double maxArmAcceleration = 150;

	public static double ElevatorConversionRatio = 0.0035390672755;
	public static double ElevatorRotationMagnetOffset = 72.158;

	public static double ElevatorKp = 0.1;

	public static double ElevatorKi = 0.0;

	public static double ElevatorKd = 0.0;

	public static double ElevatorRotationKp = 0.0;//0.03;

	public static double ElevatorRotationKi = 0.0;//0.001;

	public static double ElevatorRotationKd = 0.0;

	public static double ElevatorMinExtension = 78.74;
	public static double ElevatorMaxExtension = 157.4;

	public static double WristRotationMagnetOffset = -173.342;
	
	public static double WristKp = 10;
	public static double WristKi = 0.0;
	public static double WristKd = 0.0;

	public static double IntakeLimitMin = -90;
	public static double IntakeLimitMax = -45;

	public static int wristSoftMax = 180;
	public static int wristSoftMin = 0;

	public static double ElevatorRotationFeedforwardRatio = 0.001;

	//public static double ElevatorRotationVelocityFeedforwardRatio = 1.0/6380.0; // 

	//public static double ElevatorRotationGearRatio = (8.0/60.0)*(16.0/60.0)*(64.0/60.0);//0.03792592592

	public static int elevatorSoftMax = 78;
	public static int elevatorSoftMin = 0;

	public static int elevatorRotationSoftMax = -780;
	public static int elevatorRotationSoftMin = 2250;

	// Cubes
	public static double frontCubeHighX = 128.8;
	public static double frontCubeHighY = 28.02;

	public static double frontCubeMidX = 83.0;
	public static double frontCubeMidY = -3.0;

	public static double frontCubeLowX = 63.5;
	public static double frontCubeLowY = -54.5;

	public static double frontCubeIntakeX = 27.8;
	public static double frontCubeIntakeY = -73.7;

	public static double backCubeHighX = -128.8;
	public static double backCubeHighY = 28.02;

	public static double backCubeMidX = -83.0;
	public static double backCubeMidY = -3.0;

	public static double backCubeLowX = -63.5;
	public static double backCubeLowY = -54.5;

	public static double backCubeIntakeX = -70.5;
	public static double backCubeIntakeY = -67.5;

	// Cones
	public static double frontConeHighX = 142.820453;
	public static double frontConeHighY = 38.581881;

	public static double frontConeMidX = 99.422856;
	public static double frontConeMidY = 7.6424639;

	public static double frontConeLowX = 72.864637;
	public static double frontConeLowY = -50.203040;

	public static double frontConeIntakeX = 69.62155243;
	public static double frontConeIntakeY = -55.0238769;

	public static double backConeHighX = -128.8;
	public static double backConeHighY = 28.02;

	public static double backConeMidX = -83.0;
	public static double backConeMidY = -3.0;

	public static double backConeLowX = -63.5;
	public static double backConeLowY = -54.5;

	public static double backConeIntakeX = -72.86;
	public static double backConeIntakeY = -54.20;

	public static double backConeHumanPlayerX = 0;
	public static double backConeHumanPlayerY = 0;

	public static double backCubeHumanPlayerX = 0;
	public static double backCubeHumanPlayerY = 0;

	public static double frontConeHumanPlayerX = 0;
	public static double frontConeHumanPlayerY = 0;

	public static double frontCubeHumanPlayerX = 0;
	public static double frontCubeHumanPlayerY = 0;
}
