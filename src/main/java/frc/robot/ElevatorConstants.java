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
public static int elevatorSoftMax = 0;
public static int elevatorSoftMin = 0;

public static int elevatorRotationSoftMax = 0;
public static int elevatorRotationSoftMin = 0;

//Cubes
public static double frontCubeHighX = 0;
public static double frontCubeHighY = 0;

public static double frontCubeMidX = 0;
public static double frontCubeMidY = 0;

public static double frontCubeLow = 0;
public static double frontCubeIntake = 0;

public static double backCubeHigh = 0;
public static double backCubeMid = 0;
public static double backCubeLow = 0;
public static double backCubeIntake = 0;

//Cones
public static double frontConeHigh = 0;
public static double frontConeMid = 0;
public static double frontConeLow = 0;
public static double frontConeIntake = 0;

public static double backConeHigh = 0;
public static double backConeMid = 0;
public static double backConeLow = 0;
public static double backConeIntake = 0;

}
