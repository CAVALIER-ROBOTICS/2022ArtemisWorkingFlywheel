// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interpolation.InterpolatingTable;

/** Add your docs here. */
public class Limelight {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-cavbots");

    public static NetworkTableEntry tx = table.getEntry("tx");
    public static NetworkTableEntry ty = table.getEntry("ty");
    public static NetworkTableEntry ta = table.getEntry("ta");
    public static NetworkTableEntry tv = table.getEntry("tv");

    public static double x;
    public static double y;
    public static double area;
    public static double v;
    
    public Limelight() {
       
    }

    public static void updateValues() {
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");

        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v =tv.getDouble(0.0);

        SmartDashboard.putNumber("limelight Distance", getDistance());
        SmartDashboard.putNumber("Limelight angle", getY());
    }

    public static boolean hasTarget() {
        return v>0;
    }

    public static double getX()
    {
        return -y;
    }

    public static double getY()
    {
        return -x;
    }

    public static double getArea()
    {
        return area;
    }

    public static double getDistance() {
        return ((2.74-.762) / (Math.tan(Math.toRadians(34+getY()))));
    }

    public static double getRPM()
    {
        return InterpolatingTable.getShotParameter(getDistance()).getRPM();
    }

    public static double getAngle()
    {
        return InterpolatingTable.getShotParameter(getDistance()).getHoodAngle();
    }
}
