// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
  
public IDConstants() {
        super("//lvuser//IDConstants.ini", false);
}

public int FLDriveID = 0;
public String FLSDriveCanID ="";

public int FLSteeringID = 0;
public String FLSteeringCanID ="";

public int FRDriveID = 0;
public String FRDriveCanID ="";

public int FRSteeringID = 0;
public String FRSteeringCanID ="";

public int BLDriveID = 0;
public String BLDriveCanID ="";

public int BLSteeringID = 0;
public String BLSteeringCanID ="";

public int BRDriveID = 0;
public String BRDriveCanID ="";

public int BRSteeringID = 0;
public String BRSteeringCanID ="";

public int IntakeMotorID = 0;
public String IntakeMotorCanID ="";

public int ElevatorMotorID = 0;
public String ElevatorMotorCanID ="";

public int PDHID = 0;
public String PDHCanID ="";

public int PHID = 0;
public String PHCanID ="";


}
