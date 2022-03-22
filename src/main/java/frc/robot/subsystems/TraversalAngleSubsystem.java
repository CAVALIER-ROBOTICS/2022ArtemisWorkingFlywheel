// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TraversalAngleSubsystem extends SubsystemBase {
  /** Creates a new TraversalAngleSubsystem. */

  private CANSparkMax angle = new CANSparkMax(Constants.traveralMoveID, MotorType.kBrushless);
  private RelativeEncoder angleEnc = angle.getEncoder();
  private SparkMaxPIDController anglePID = angle.getPIDController();

  public TraversalAngleSubsystem() {
    angle.restoreFactoryDefaults();
    angle.setOpenLoopRampRate(2);
    angle.setIdleMode(IdleMode.kBrake);
    angle.setInverted(true);

    anglePID.setP(0.1);
    anglePID.setI(0);
    anglePID.setD(0);
    // anglePID.setOutputRange(-.2, .2); 
  }

  public void setAngle(double x) {
    angle.setVoltage(x);
  }
  
  public void setAnglePos() {
    anglePID.setReference(angleEnc.getPosition(), CANSparkMax.ControlType.kPosition);
    // SmartDashboard.putNumber("angleEnc", angleEnc.getPosition());
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("AngleClimb output", angle.get());
  }
}