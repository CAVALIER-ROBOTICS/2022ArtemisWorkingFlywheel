// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TraversalAngleSubsystem;

public class LowerTraversalCommand extends CommandBase {
  /** Creates a new LowerTraversalCommand. */
  TraversalAngleSubsystem traversalSub;
  public LowerTraversalCommand(TraversalAngleSubsystem t) {
    // Use addRequirements() here to declare subsystem dependencies.
    traversalSub = t;
    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    traversalSub.setAngle(-3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    traversalSub.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return traversalSub.getVoltage()>15;
  }
}
