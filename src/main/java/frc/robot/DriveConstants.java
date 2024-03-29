// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class DriveConstants extends BBConstants {

    public static double wheelTrackWidth = 0.0; // In meters
    public static double wheelBase = 0.0; // In meters
    public static double wheelDiameter = 0.0; // In meters
    public static double driveGearReduction = 0.0; // driving gear/driven gear
    public static double steerGearReduction = 0.0; // driving gear/driven gear

    public static double FLSteerOffset = 0.0;
    public static double FRSteerOffset = 0.0;
    public static double BLSteerOffset = 0.0;
    public static double BRSteerOffset = 0.0;

    public static double TranslationXkP = 20;
    public static double TranslationXkI = 3;
    public static double TranslationXkD = 0.0;
    public static double TranslationXkF = 1.7;

    public static double TranslationYkP = 20;
    public static double TranslationYkI = 3;
    public static double TranslationYkD = 0.0;
    public static double TranslationYkF = 1.7;

    public static double RotationkP = 1;
    public static double RotationkI = 0.0;
    public static double RotationkD = 0.07;
    public static double RotationkF = 0.0;
    
    public static double driveSpeedMultiplier = 0.5;
    public static double rotationSpeedMultiplier = 0.5;

    public DriveConstants() {
        super("/home/lvuser/DriveConstants.ini", true);
    }
}
