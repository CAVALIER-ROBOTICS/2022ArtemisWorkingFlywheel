// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  /** Creates a new ShootCommand. */
  ShooterSubsystem shootSub;
  public ShootCommand(ShooterSubsystem s) {
    // Use addRequirements() here to declare subsystem dependencies.
    shootSub = s;
    addRequirements(s);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("table RPM", Limelight.getRPM());
    shootSub.setShooterVelocity(Limelight.getRPM());

    // if(Math.abs(Limelight.getRPM()-shootSub.getVolicty())<50) {
    //   RobotContainer.operator.setRumble(RumbleType.kLeftRumble, 1.0);
    //   RobotContainer.operator.setRumble(RumbleType.kRightRumble, 1.0);
    // }
    // else{
    //   RobotContainer.operator.setRumble(RumbleType.kLeftRumble, 0.0);
    //   RobotContainer.operator.setRumble(RumbleType.kRightRumble, 0.0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    shootSub.setShooter(0.0);
    // shootSub.setShooter(0);
    // RobotContainer.operator.setRumble(RumbleType.kLeftRumble, 0.0);
    // RobotContainer.operator.setRumble(RumbleType.kRightRumble, 0.0);
    // shootSub.setShooterVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
