// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RightClimbSubsystem;

public class RightClimbCommand extends CommandBase {
  /** Creates a new RightClimbCommand. */
  RightClimbSubsystem right;
  DoubleSupplier xtrans;
  public RightClimbCommand(RightClimbSubsystem r, DoubleSupplier x) {
    // Use addRequirements() here to declare subsystem dependencies.
    right = r;
    xtrans = x;
    addRequirements(r);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xtrans.getAsDouble()>0.1||xtrans.getAsDouble()<-.1)
    {
      right.set(xtrans.getAsDouble());
    }
    else {
      right.setPos();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
