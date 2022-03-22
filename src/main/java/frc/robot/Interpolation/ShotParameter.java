// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interpolation;

/** Add your docs here. */
public class ShotParameter 
{
    public final double hoodAngle;
    public final double rpm;
    public final double offset;

    // Constructor
    public ShotParameter(double h, double r, double o) {
        hoodAngle = h;
        rpm = r;
        offset = o;
    }   

    public double getRPM()
    {
        return rpm;
    }

    public double getHoodAngle()
    {
        return hoodAngle;
    }

    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(hoodAngle, end.hoodAngle, t), 
            lerp(rpm, end.rpm, t), 
            lerp(offset, end.offset, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
}
