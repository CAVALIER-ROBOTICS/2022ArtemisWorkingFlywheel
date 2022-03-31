package frc.robot.subsystems;


public interface DriveTrainConstants {

  //USE CHARACTERIZATION TOOL FOR THIS INFO https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
  public static final double maxVoltage = 12;//7


  // this is just an estimate In radians
  public static final double maxVelocityPerSecond = 8;//2

  public static final double maxAngularVelocityPerSecond = maxVelocityPerSecond/Math.hypot(0.4041,.4041);
  //when upside down, decreasing offset = turn left/counterclockwise, increasing offset = turn right/clockwise
//If it's negative increase
  public static final int pigeonID = 21;

  public static final int frontLeftDriveMotor = 2; // FIXME Set front left module drive motor ID
  public static final int frontLeftSteerMotor = 6; // FIXME Set front left module steer motor ID
  public static final int frontLeftSteerEncoder = 8; // FIXME Set front left steer encoder ID
  public static final double frontLeftModuleSteerOffset = -Math.toRadians(347.5); // FIXME Measure and set front left steer offset

  public static final int frontRightDriveMotor = 3; // FIXME Set front right drive motor ID
  public static final int frontRightSteerMotor = 7; // FIXME Set front right steer motor ID
  public static final int frontRightSteerEncoder = 11; // FIXME Set front right steer encoder ID
  public static final double frontRightModuleSteerOffset = -Math.toRadians(120); // FIXME Measure and set front right steer offset

  public static final int backLeftDriveMotor = 1; // FIXME Set back left drive motor ID
  public static final int backLeftSteerMotor = 5; // FIXME Set back left steer motor ID
  public static final int backLeftSteerEncoder = 9; // FIXME Set back left steer encoder ID
  public static final double backLeftModuleSteerOffset = -Math.toRadians(274.75); // FIXME Measure and set back left steer offset

  public static final int backRightDriveMotor = 24; // FIXME Set back right drive motor ID
  public static final int backRightSteerMotor = 4; // FIXME Set back right steer motor ID
  public static final int backRightSteerEncoder= 10; // FIXME Set back right steer encoder ID
  public static final double backRightModuleSteerOffset = -Math.toRadians(262); // FIXME Measure and set back right steer offset


  public static final int volts = 12;
  public static final int voltsSecondsPerMeter = 7;
  public static final int voltSecondsSquaredPerMeter = 5;


}