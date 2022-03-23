// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RightClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  CANSparkMax rightClimb = new CANSparkMax(Constants.rightClimb,MotorType.kBrushless);
  RelativeEncoder rightEnc = rightClimb.getEncoder();
  SparkMaxPIDController rightPID = rightClimb.getPIDController();

  public RightClimbSubsystem() {
    rightClimb.restoreFactoryDefaults();
    rightClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setInverted(false);
    // rightClimb.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65360);

    rightPID.setP(.1);
    rightPID.setI(0);
    rightPID.setD(0);
    // rightPID.setOutputRange(-.2, .2);

    rightClimb.setOpenLoopRampRate(1); 
  }

  public void set(double x)
  {
    rightClimb.set(x);
  }

  public double getVolt() {
    return rightClimb.getOutputCurrent();
  }

  public void stop() {
    rightClimb.stopMotor();
  }

  public void setPos() {
    rightPID.setReference(rightEnc.getPosition(), CANSparkMax.ControlType.kPosition);
    // SmartDashboard.putNumber("rightCLimbEnc", rightEnc.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("rightClimb output", rightClimb.get());

  }
}
