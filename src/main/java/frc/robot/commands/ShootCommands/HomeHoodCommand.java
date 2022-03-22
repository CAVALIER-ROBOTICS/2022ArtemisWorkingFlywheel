// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HomeHoodCommand extends CommandBase {
  /** Creates a new HomeHoodCommand. */
  HoodSubsystem hoodSub;
  public HomeHoodCommand(HoodSubsystem h) {
    // Use addRequirements() here to declare subsystem dependencies.
    hoodSub = h;
    addRequirements(h);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodSub.setHood(.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Hood Voltage", hoodSub.getVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSub.setHood(0);
    hoodSub.setEncoder(42.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hoodSub.getVoltage()<23){
      return false;
    }
    else{
      return true;
    }
  }
}
