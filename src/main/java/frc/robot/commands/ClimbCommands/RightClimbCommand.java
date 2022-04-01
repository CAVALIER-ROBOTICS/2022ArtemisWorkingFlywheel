// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RightClimbSubsystem;

public class RightClimbCommand extends CommandBase {
  /** Creates a new RightClimbCommand. */
  RightClimbSubsystem rightClimb;
  BooleanSupplier upButton;
  BooleanSupplier downButton;

  public RightClimbCommand(RightClimbSubsystem r, BooleanSupplier up, BooleanSupplier down) {
    // Use addRequirements() here to declare subsystem dependencies.
    rightClimb = r;
    upButton = up;
    downButton = down;
    addRequirements(r);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(upButton.getAsBoolean()) {
      rightClimb.set(.7);
    }
    else if(downButton.getAsBoolean()) {
      rightClimb.set(-.6);
    }
    else {
      rightClimb.setPos();
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
