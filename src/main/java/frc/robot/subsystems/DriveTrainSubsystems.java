// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainSubsystems extends SubsystemBase implements DriveTrainConstants {
  /** Creates a new DriveTrainSubsystems. */


  // search up Pigeon IMU for more info
  WPI_Pigeon2 pidgey = new WPI_Pigeon2(pigeonID);
  // Odometry class for tracking robot pose  Rotation2d.fromDegrees(getFusedHeading());     getRotation2d()           Rotation2d.fromDegrees(pidgey.getCompassHeading()
    private final SwerveDriveOdometry odo = new SwerveDriveOdometry(Constants.m_kinematics, pidgey.getRotation2d());

    // These are the modules initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private double xVelocity;
    private double yVelocity;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);

    Field2d field = new Field2d();


  public DriveTrainSubsystems() {
    SmartDashboard.putData("Field", field);

    

    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      // This can either be STANDARD or FAST depending on your gear configuration
      Mk4SwerveModuleHelper.GearRatio.L2,
      // This is the ID of the drive motor
      frontLeftDriveMotor,
      // This is the ID of the steer motor
      frontLeftSteerMotor,
      // This is the ID of the steer encoder
      frontLeftSteerEncoder,
      // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
      frontLeftModuleSteerOffset);

    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
      Mk4SwerveModuleHelper.GearRatio.L2,
      frontRightDriveMotor,
      frontRightSteerMotor,
      frontRightSteerEncoder,
      frontRightModuleSteerOffset
    );

      backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        Mk4SwerveModuleHelper.GearRatio.L2,
        backLeftDriveMotor,
        backLeftSteerMotor,
        backLeftSteerEncoder,
        backLeftModuleSteerOffset
      );

      backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        // Shuffleboard.getTab("SwerveData").getLayout("SwerveData"),
        Mk4SwerveModuleHelper.GearRatio.L2,
        backRightDriveMotor,
        backRightSteerMotor,
        backRightSteerEncoder,
        backRightModuleSteerOffset
      );
  }

  public void zeroGyroscope() {
    //zeros gyroscope
    pidgey.reset();
  }

  public Rotation2d getGyroscopeRotation() 
  {
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - pidgey.getAngle()); 
  }

  public void drive(ChassisSpeeds speeds)
  {
    states = Constants.m_kinematics.toSwerveModuleStates(speeds);
    // states = Constants.m_kinematics.toSwerveModuleStates(speeds);
    frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage, states[3].angle.getRadians());
  }

  public void setModules(SwerveModuleState[] speeds)
  {
    states = speeds;
    // drive(Constants.m_kinematics.toChassisSpeeds(speeds));
    frontLeftModule.set(speeds[0].speedMetersPerSecond * 5, speeds[0].angle.getRadians());
    frontRightModule.set(speeds[1].speedMetersPerSecond * 5, speeds[1].angle.getRadians());
    backLeftModule.set(speeds[2].speedMetersPerSecond * 5, speeds[2].angle.getRadians());
    backRightModule.set(speeds[3].speedMetersPerSecond * 5, speeds[3].angle.getRadians());
  }

  @Override
  public void periodic() {
  }

  public SwerveModuleState[] invert(SwerveModuleState[] x) {
    SwerveModuleState[] temp = new SwerveModuleState[4];
    for(int i = 0; i <4; i++) {
      temp[i] = new SwerveModuleState((x[i].speedMetersPerSecond),x[i].angle);
    }
    return temp;
  }

  public void updateOdo() {
    // SwerveModuleState[] temp = invert(states);
    odo.update(pidgey.getRotation2d(), states[0],states[1],states[2],states[3]);
    field.setRobotPose(getPose());
    // SmartDashboard.putString("Odo", ""+odo.getPoseMeters());
  }

  private double unitsToDistance(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / 6.75;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2));
    return positionMeters;
  }

  // 0.10033
  // (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
  // (15.0 / 32.0) * (10.0 / 60.0)

  public Pose2d getPose() {
   return odo.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odo.resetPosition(pose, pidgey.getRotation2d());
  }

  public void fieldOrientedDrive(DoubleSupplier xtrans, DoubleSupplier ytrans, DoubleSupplier rot) {
    drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xtrans.getAsDouble(),
        ytrans.getAsDouble(),
        rot.getAsDouble(),
        getGyroscopeRotation()));
    xVelocity = xtrans.getAsDouble();
    yVelocity = ytrans.getAsDouble();
  }
  
  public void robotOrientedDrive(DoubleSupplier xtrans, DoubleSupplier ytrans, DoubleSupplier rot) {
    drive(
      new ChassisSpeeds(
        xtrans.getAsDouble(),
        ytrans.getAsDouble(),
        rot.getAsDouble()));
    xVelocity = xtrans.getAsDouble();
    yVelocity = ytrans.getAsDouble();
  }

  public double getSpeed()
  {
    double speed = Math.sqrt(Math.pow(2, xVelocity) + Math.pow(2, yVelocity));
    return speed;
  }
}
 