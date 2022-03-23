// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TraversalAngleSubsystem;

public class TraversalAngleCommand extends CommandBase {
  /** Creates a new TraversalAngleCommand. */
  TraversalAngleSubsystem traverseSub;
  DoubleSupplier ytrans;

  public TraversalAngleCommand(TraversalAngleSubsystem t, DoubleSupplier y) {
    // Use addRequirements() here to declare subsystem dependencies.
    traverseSub = t;
    ytrans = y;
    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ytrans.getAsDouble()>0.5||ytrans.getAsDouble()<-0.5) {
      traverseSub.setAngle(ytrans.getAsDouble());
    }
    else {
      traverseSub.setAnglePos();
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
