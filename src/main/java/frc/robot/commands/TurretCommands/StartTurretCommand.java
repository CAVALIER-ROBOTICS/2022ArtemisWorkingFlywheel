// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class StartTurretCommand extends CommandBase {
  /** Creates a new StartTurret. */
  TurretSubsystem turretSub;
  public StartTurretCommand(TurretSubsystem t) {
    // Use addRequirements() here to declare subsystem dependencies.
    turretSub = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    turretSub.setOpenLoop(.15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    turretSub.setOpenLoop(0);
    turretSub.softLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turretSub.getPos()>10)
    {
      return true;
    }
    return false;
  }
}
