// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LeftClimbSubsytem;
import frc.robot.subsystems.RightClimbSubsystem;

public class LeftClimbCommand extends CommandBase {
  /** Creates a new LeftClimbCommand. */
  LeftClimbSubsytem left;
  DoubleSupplier xtrans;
  
  public LeftClimbCommand(LeftClimbSubsytem r, DoubleSupplier x) {
    // Use addRequirements() here to declare subsystem dependencies.
    left = r;
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
      left.set(xtrans.getAsDouble());
    }
    else {
      left.setPos();
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
