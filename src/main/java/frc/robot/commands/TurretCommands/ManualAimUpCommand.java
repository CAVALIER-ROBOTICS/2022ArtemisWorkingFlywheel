// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.TurretSubsystem;

public class ManualAimUpCommand extends CommandBase {
  /** Creates a new ManualAimRightCommand. */
  TurretSubsystem turretSub;

  public ManualAimUpCommand(TurretSubsystem t) {
    // Use addRequirements() here to declare subsystem dependencies.
    turretSub = t;
    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSub.manualAimUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Limelight.hasTarget();  
  }
}
