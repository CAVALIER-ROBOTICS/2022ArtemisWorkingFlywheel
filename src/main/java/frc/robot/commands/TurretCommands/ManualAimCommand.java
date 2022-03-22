// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.TurretCommands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Limelight;
// import frc.robot.subsystems.TurretSubsystem;

// public class ManualAimCommand extends CommandBase {
//   /** Creates a new ManualAimCommand. */
//   TurretSubsystem turretSub;
//   DoubleSupplier xtrans;
//   public ManualAimCommand(TurretSubsystem t, DoubleSupplier x) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     turretSub = t;
//     xtrans=x;
//     addRequirements(t);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//       turretSub.manualAim(-xtrans.getAsDouble()/2);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Limelight.hasTarget();
//   }
// }
