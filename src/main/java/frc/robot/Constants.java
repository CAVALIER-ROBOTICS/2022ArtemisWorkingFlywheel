// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // 65535ms  is the periodic frame status stuff 
 
public final class Constants {

    public static final String[] fourBallAuto = { 
        "FourBallPath1",
        "FourBallPath2"
     };

    public static final String simpleAuto = "SimpleAuto";

    public static final int turretID = 12;
    public static final int floorID = 18;
    public static final int intakeID = 20;
    public static final int trasversalID = 19;
    public static final int traveralMoveID = 17;
    public static final int kickerID = 13;
    public static final int leftShootID = 15;
    public static final int rightShooterID = 14;
    public static final int hoodID = 16;
    public static final int rightClimb = 23;
    public static final int leftClimb = 22;



    public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(.4041, .4041),
        // Front right
        new Translation2d(.4041, -.4041),
        // Back left
        new Translation2d(-.4041,.4041),
        // Back right
        new Translation2d(-.4041, -.4041));

        public static final int encoderCPR = 2048;
        // inches
        public static final double wheelDiamter = .1016;
        public static final double distancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (wheelDiamter * Math.PI) / (double) encoderCPR;

        public static double acceptedVolts = 65;

        public static final double TranslationController = 1;//.7
        public static final double StrafeController = 1;//.7
        public static final double ThetaController = 1;//1

        //per second radians                          Math.PI;
        public static final double maxAngularSpeed = 3;
        // per second per second in radians
        public static final double maxAngularAcceleration = 2;

        public static final TrapezoidProfile.Constraints thetaControllerConstraints = 
        new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAcceleration);
}
