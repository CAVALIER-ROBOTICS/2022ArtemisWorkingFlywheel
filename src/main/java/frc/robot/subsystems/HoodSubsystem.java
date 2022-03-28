// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  CANSparkMax hoodMotor = new CANSparkMax(Constants.hoodID,MotorType.kBrushless);
  RelativeEncoder hoodEnc = hoodMotor.getEncoder();
  //Max encoder value = 44
  //Min encoder value = 0
  
  public HoodSubsystem() {
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setInverted(true);
    // hoodEnc.setPosition(0);
    // hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65500);
    // hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);
    // hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65510);
    hoodEnc.setPosition(0);
  }

  public void setHood(double x)
  {
    hoodMotor.set(x);
  }
  public void setEncoder(double x){
    hoodEnc.setPosition(x);
  }
  public double getPos() {
    return hoodEnc.getPosition();
  }

  public double getAngle() {
    return getPos()*30/41+14;
  }

  public double getVoltage() {
    return hoodMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hood angle", getAngle());
    // SmartDashboard.putNumber("hood encoder", getPos());

  }
}
