// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import java.awt.geom.Point2D;


/** Add your docs here. */
public class ArmPosition {
    private Rotation2d rotation;
    private double extension;
    public ArmPosition(Rotation2d rotation, double extension)
    {
        this.rotation = rotation;
        this.extension = extension;
    }
    public double getExtension()
    {
        return extension;
    }
    public Rotation2d getRotation()
    {
        return rotation;
    }
    public Point2D.Double polarToXY()
    {
        double x = Math.cos(this.getRotation().getRadians()) * this.getExtension();
        double y = Math.sin(this.getRotation().getRadians()) * this.getExtension();
        return new Point2D.Double(x, y);
    }
    
}
