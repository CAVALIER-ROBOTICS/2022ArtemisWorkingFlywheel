// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystems;

public class RobotDriveCommand extends CommandBase {
  /** Creates a new RobotDriveCommand. */
  DriveTrainSubsystems driveSub;
  DoubleSupplier translationXSupplier;
  DoubleSupplier translationYSupplier;
  DoubleSupplier rotationSupplier;
  
  public RobotDriveCommand(DoubleSupplier xtrans, DoubleSupplier ytrans, DoubleSupplier rot, DriveTrainSubsystems d) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSub = d;
    translationXSupplier = xtrans;
    translationYSupplier = ytrans;
    rotationSupplier = rot;

    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.drive(
      new ChassisSpeeds(
            translationXSupplier.getAsDouble(),
            translationYSupplier.getAsDouble(),
            rotationSupplier.getAsDouble())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
