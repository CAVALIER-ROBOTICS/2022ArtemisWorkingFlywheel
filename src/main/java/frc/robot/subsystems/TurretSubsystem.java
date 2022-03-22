// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  CANSparkMax turret = new CANSparkMax(Constants.turretID, MotorType.kBrushless);
  RelativeEncoder encoder = turret.getEncoder();
  SparkMaxPIDController turretPID = turret.getPIDController();
  public int acceptedVolts = 30;

  public double kP, kI, kD, kIz, kFF;
  
  boolean turnUp;
  boolean turnDown;

  double desiredAngle;
  
  public TurretSubsystem() 
  {
    turret.restoreFactoryDefaults();
    turret.setIdleMode(IdleMode.kBrake);
    turret.setInverted(false);
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    
    // turret.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 41);
    // turret.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);

    turret.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 64200);

    kP = 0.08; 
    kI = 0.000;
    kD = 0; 
    kIz = 0; 
    kFF =  0; //0.001; 

    // set PID coefficients
    turretPID.setP(kP);
    turretPID.setI(kI);
    turretPID.setD(kD);
    turretPID.setIZone(kIz);
    turretPID.setFF(kFF);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);

    turnUp = false;
    turnDown = false;

  }

  public void softLimit(boolean x)
  {
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, x);
    // turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, x);
  }

  public double getPos() {
    return encoder.getPosition();
  }

  public double getAngle() {
    return  (getPos() / 50) * 360;
  }

  public void setDesiredAngle(double angle) {
    turretPID.setReference(angle/360*50, ControlType.kPosition);
  }

  public void setOpenLoop(double volts) {
    turretPID.setReference(volts, ControlType.kDutyCycle);
  }

  public void resetPos() {
    encoder.setPosition(0.0);
  }

  private double getError() {
      return getAngle() - desiredAngle;
  }

  public boolean inRange() {
    return (getAngle()>90 || getAngle()<213);
  }

  public void updateTurns()
  {
    if(getAngle()<10) {turnUp = true;}
    
    if(getAngle()>290) {turnDown = true;}

    if(getAngle()>150&& turnUp) {turnUp = false;}

    if(getAngle()<150 && turnDown) {turnDown = false;}
  }

  public void aim()
  {
    updateTurns();
    if(inRange()&&!turnDown&&!turnUp) {
      setDesiredAngle(desiredAngle);
    }
    else {
      if(turnUp) { 
        setOpenLoop(0.3);
      }
      if(turnDown) {
        setOpenLoop(-0.3);
      }
    }
  }

  // public void manualAim(double volt) {
  //   if(inRange()) {
  //     setOpenLoop(volt);
  //   }
  //   else {
  //     if(getAngle()>290 && volt<0) { 
  //       setOpenLoop(volt);
  //     }
  //     if(getAngle()<10 && volt>0) {
  //       setOpenLoop(volt);
  //     }    
  //   }
  //   SmartDashboard.putNumber("Manual aim angle", volt);
  // }

  public void manualAimUp() {
    if(getAngle()<280) {
      setOpenLoop(.2);
    }
  }

  public void manualAimDown() {
    if(getAngle()>20) {
      setOpenLoop(-.2);
    }
  }

  public void stop() {
      setOpenLoop(0);
  }

  public double getVolts() {
    return turret.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Turret Enc", encoder.getPosition());
    // SmartDashboard.putBoolean("turnUp",turnUp);
    // SmartDashboard.putBoolean("turnDown",turnDown);
    // SmartDashboard.putNumber("turret voltage", getVolts());

    // SmartDashboard.putNumber("Desired Angle", desiredAngle);
    // SmartDashboard.putNumber("curretn angle", getAngle());
    // SmartDashboard.putNumber("limelightX", Limelight.getX());

    desiredAngle = getAngle() + Limelight.getX();
  }
}
