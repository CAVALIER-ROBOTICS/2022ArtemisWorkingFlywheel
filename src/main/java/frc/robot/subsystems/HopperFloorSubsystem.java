// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperFloorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax hopperFloor = new CANSparkMax(Constants.floorID, MotorType.kBrushless);
  
  public HopperFloorSubsystem() 
  {
    hopperFloor.restoreFactoryDefaults();
    hopperFloor.setIdleMode(IdleMode.kCoast);
    hopperFloor.setInverted(false);
    // hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65400);
    hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65200);
    hopperFloor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65210);
    // hopperFloor.setInverted(false);

  }

  public void setFloor(double x){
    hopperFloor.setVoltage(x);
  }

  public void stopFloor() {
    hopperFloor.stopMotor();
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
