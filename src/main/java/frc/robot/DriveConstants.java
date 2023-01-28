// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class DriveConstants extends BBConstants{

    public double wheelTrackWidth = 0.0;   //In meters
    public double wheelBase = 0.0;  //In meters
    public double wheelDiameter = 0.0;  //In meters
    public double driveGearReduction = 0.0; // driving gear/driven gear
    public double steerGearReduction = 0.0; // driving gear/driven gear
    public double moduleGearRatio = 0.0;
    
    public double FLSteerOffset = 0.0;
    public double FRSteerOffset = 0.0;
    public double BLSteerOffset = 0.0;
    public double BRSteerOffset = 0.0;


    public DriveConstants() {
        super("//lvuser//DriveConstants.ini", true);
    }}

