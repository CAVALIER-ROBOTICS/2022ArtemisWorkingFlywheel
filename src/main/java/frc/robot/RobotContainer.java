// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.AutoAimCommand;
import frc.robot.Autonomous.AutoIntakeCommand;
import frc.robot.Autonomous.AutoKickerCommand;
import frc.robot.Autonomous.AutonSetUpCommand;
import frc.robot.Autonomous.DriveAuto;
import frc.robot.commands.ClimbCommands.HoldLeftCommand;
import frc.robot.commands.ClimbCommands.HoldRightCommand;
import frc.robot.commands.ClimbCommands.HoldTraversalAngleCommand;
import frc.robot.commands.ClimbCommands.HoldTraversalCommand;
import frc.robot.commands.ClimbCommands.InTraversalClimbCommand;
import frc.robot.commands.ClimbCommands.LeftClimbCommand;
import frc.robot.commands.ClimbCommands.OutTraversalClimbCommand;
import frc.robot.commands.ClimbCommands.RightClimbCommand;
import frc.robot.commands.ClimbCommands.TraversalAngleCommand;
import frc.robot.commands.ClimbCommands.TraversalClimbCommand;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.RobotDriveCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OutTakeCommand;
import frc.robot.commands.ShootCommands.HomeHoodCommand;
import frc.robot.commands.ShootCommands.HoodCommand;
import frc.robot.commands.ShootCommands.KickCommand;
import frc.robot.commands.ShootCommands.ShootCommand;
import frc.robot.commands.TurretCommands.AimCommand;
import frc.robot.commands.TurretCommands.ManualAimDownCommand;
import frc.robot.commands.TurretCommands.ManualAimUpCommand;
import frc.robot.commands.TurretCommands.StartTurretCommand;
import frc.robot.commands.TurretCommands.TurnTurretCommand;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperFloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LeftClimbSubsytem;
import frc.robot.subsystems.RightClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TraversalAngleSubsystem;
import frc.robot.subsystems.TraversalClimbSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);


  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();
  TurretSubsystem turretSub = new TurretSubsystem();;
  HopperFloorSubsystem floorSub = new HopperFloorSubsystem();
  IntakeSubsystem intakeSub = new IntakeSubsystem();
  ShooterSubsystem shooterSub = new ShooterSubsystem();
  HoodSubsystem hoodSub = new HoodSubsystem();
  KickerSubsystem kickSub = new KickerSubsystem();
  DriveAuto autoDrive = new DriveAuto(driveSub);
  TraversalClimbSubsystem traversalClimbSub = new TraversalClimbSubsystem();
  RightClimbSubsystem rightClimb = new RightClimbSubsystem();
  LeftClimbSubsytem leftClimb = new LeftClimbSubsytem();
  TraversalAngleSubsystem traversalAngleSub = new TraversalAngleSubsystem();

  // PathPlannerTrajectory path1;
  // PathPlannerTrajectory path2;
  PathPlannerTrajectory path;

  Trigger leftClimbDown = new Trigger(()-> getLeftTrigger());
  Trigger rightClimbDown = new Trigger(()-> getRightTrigger());

  Trigger leftClimbUp = new Trigger(()-> getOperatorLeftBumper());
  Trigger rightClimbUp = new Trigger(()-> getOperatorRightBumper());

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SmartDashboard.putNumber("Hood Angle input", 14);
    // SmartDashboard.putNumber("RPM input", 1800);
    // path1 = PathPlanner.loadPath("ComplexAutoPath1", 2, 1);
    // path2 = PathPlanner.loadPath("ComplexAutoPath2", 2, 1);

    // Configure the button bindings
    path = PathPlanner.loadPath("SimpleAutoPath", 2, 1);

    configureButtonBindings();
 
    

    turretSub.setDefaultCommand( new SequentialCommandGroup(
      new TurnTurretCommand(turretSub),
      new StartTurretCommand(turretSub),
      new AimCommand(turretSub)));

    hoodSub.setDefaultCommand(
      // new HomeHoodCommand(hoodSub),
      new HoodCommand(hoodSub));

    // traversalSub.setDefaultCommand(
    //   new TraversalClimbCommand(
    //     traversalSub,
    //     ()->  modifyAxis(operator.getRightX()), 
    //     ()-> modifyAxis(operator.getRightY())));

    traversalAngleSub.setDefaultCommand(new TraversalAngleCommand(traversalAngleSub,()-> operator.getRightY()*-5));
    traversalClimbSub.setDefaultCommand(new TraversalClimbCommand(traversalClimbSub,()-> operator.getLeftY()*5));

    leftClimb.setDefaultCommand(new LeftClimbCommand(leftClimb, ()-> leftClimbUp.getAsBoolean(), ()-> leftClimbDown.getAsBoolean()));
    rightClimb.setDefaultCommand(new RightClimbCommand(rightClimb, ()->rightClimbUp.getAsBoolean(), ()->rightClimbDown.getAsBoolean()));

    // leftClimb.setDefaultCommand(new HoldLeftCommand(leftClimb));
    // rightClimb.setDefaultCommand(new HoldRightCommand(rightClimb));

    //passes conditional command into the default command of drive
    driveSub.setDefaultCommand(
      new FieldDriveCommand(
        () -> modifyAxis(driver.getLeftY()) * DriveTrainSubsystems.maxVelocityPerSecond,
        () -> modifyAxis(driver.getLeftX()) * DriveTrainSubsystems.maxVelocityPerSecond,
        () -> modifyAxis(driver.getRightX()) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
        driveSub
      ));

    // rightClimb.setDefaultCommand(new RightClimbCommand(rightClimb, ()->operator.getRightY()));
    // leftClimb.setDefaultCommand(new LeftClimbCommand(leftClimb, ()->operator.getLeftY()));

    // traversalAngleSub.setDefaultCommand(new HoldTraversalAngleCommand(traversalAngleSub));
    // traversalClimbSub.setDefaultCommand(new HoldTraversalCommand(traversalClimbSub));

        
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    JoystickButton intake = new JoystickButton(driver, 5);
    JoystickButton outake = new JoystickButton(driver, 6);
    JoystickButton reset = new JoystickButton(driver, 4);
    JoystickButton changeDrive = new JoystickButton(driver, 3);
    JoystickButton shoot = new JoystickButton(operator, 6);
    JoystickButton kicker = new JoystickButton(operator, 5);
    // JoystickButton climbUp = new JoystickButton(operator, 4);
    // JoystickButton climbDown = new JoystickButton(operator, 1);
    // JoystickButton moveClimbBack = new JoystickButton(operator, 2);
    // JoystickButton moveClimbForward = new JoystickButton(operator, 3);

    // Trigger traversalDown = new Trigger(()-> getDownDPad());
    // Trigger traversalUp = new Trigger(()-> getUpDPad());
    // Trigger angleTraversalUp = new Trigger(()-> getRightDPad());
    // Trigger angleTraversalDown = new Trigger(()-> getLeftDPad());
    


    JoystickButton aimUp = new JoystickButton(operator,3);
    JoystickButton aimDown = new JoystickButton(operator,2);
    // JoystickButton raiseHood = new JoystickButton(operator, 1);
    // JoystickButton lowerHood = new JoystickButton(operator, 4);

    // raiseHood.whileActiveContinuous(new StartEndCommand(()->hoodSub.setHood(.3), ()->hoodSub.setHood(0), hoodSub));
    // lowerHood.whileActiveContinuous(new StartEndCommand(()->hoodSub.setHood(-.3), ()->hoodSub.setHood(0), hoodSub));

    // leftClimbDown.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> leftClimb.set(-.5),
    //     ()-> leftClimb.stop(),
    //     leftClimb));
    
    // rightClimbDown.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> rightClimb.set(-.5),
    //     ()-> rightClimb.stop(), 
    //     rightClimb));

    // leftClimbUp.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> leftClimb.set(.5), 
    //     ()-> leftClimb.stop(), 
    //     leftClimb));
    
    // rightClimbUp.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> rightClimb.set(.5), 
    //     ()-> rightClimb.stop(), 
    //     rightClimb));

    


    intake.whenHeld(
      new IntakeCommand(intakeSub, floorSub));

    outake.whenHeld(
      new OutTakeCommand(intakeSub, floorSub, kickSub));
    
    // homeClimb.whenPressed(new ParallelCommandGroup(
    //   new HomeLeftClimbCommand(leftClimb),
    //   new HomeRightClimbCommand(rightClimb)
    // ));

    
    // traversalDown.whileActiveContinuous(new InTraversalClimbCommand(traversalClimbSub));
    // traversalUp.whileActiveContinuous(new OutTraversalClimbCommand(traversalClimbSub));

    // angleTraversalDown.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> traversalAngleSub.setAngle(-15),
    //     ()-> traversalAngleSub.setAngle(0), 
    //     traversalAngleSub));
    
    // angleTraversalUp.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> traversalAngleSub.setAngle(15), 
    //     ()-> traversalAngleSub.setAngle(0),
    //     traversalAngleSub));

    // climbUp.whileActiveContinuous(new ParallelCommandGroup(
    //   new StartEndCommand(
    //     () -> rightClimb.setClimb(0.8),
    //     () -> rightClimb.setClimb(0), 
    //     rightClimb),
    //   new StartEndCommand(
    //     () -> leftClimb.setClimb(0.8),
    //     () -> leftClimb.setClimb(0.0),
    //     leftClimb)));

    // climbDown.whileActiveContinuous(new ParallelCommandGroup(
    //   new StartEndCommand(
    //     () -> rightClimb.setClimb(-0.8),
    //     () -> rightClimb.setClimb(0), 
    //     rightClimb),
    //   new StartEndCommand(
    //     () -> leftClimb.setClimb(-0.8),
    //     () -> leftClimb.setClimb(0),
    //     leftClimb)
    // ));

    //rightClimb.setDefaultCommand(new RunCommand(() -> rightClimb.setClimb(()-> operator.getRightY()), rightClimb));
    // leftClimb.setDefaultCommand(new RunCommand(() -> leftClimb.setClimb(()-> operator.getLeftY()), leftClimb));

    // traversalDown.whileActiveContinuous(
    //   new ConditionalCommand(
    //     new InTraversalClimbCommand(traversalClimbSub),
    //     new HoldTraversalCommand(traversalClimbSub), 
    //     ()-> traversalDown.getAsBoolean()));
      
    // traversalUp.whileActiveContinuous(
    //   new ConditionalCommand(
    //     new OutTraversalClimbCommand(traversalClimbSub), 
    //     new HoldTraversalCommand(traversalClimbSub), 
    //     ()-> traversalUp.getAsBoolean()));

    // lowerTraversal.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> traversalClimbSub.setClimb(-10), 
    //     ()-> traversalClimbSub.setClimb(0),
    //     traversalClimbSub));

    // moveTraversalForward.whileActiveContinuous(
    //   new ConditionalCommand(
    //     new TraversalClimbCommand(traversalClimbSub),
    //     new HoldTraversalCommand(traversalClimbSub),
    //     ()-> moveTraversalForward.getAsBoolean()));

    // raiseTraversal.whileActiveContinuous(
    //   new ConditionalCommand(
    //     new TraversalClimbCommand(traversalClimbSub),
    //     new HoldTraversalCommand(traversalClimbSub),
    //     ()-> moveTraversalForward.getAsBoolean()));




    // moveTraversalBackward.whileActiveContinuous(
    //   new StartEndCommand(
    //     ()-> traversalAngleSub.setAngle(-7),
    //     ()-> traversalAngleSub.setAngle(0), 
    //     traversalAngleSub));


    

      
    
  
    aimUp.whileActiveContinuous(new ManualAimUpCommand(turretSub),true);
    aimDown.whileActiveContinuous(new ManualAimDownCommand(turretSub),true);

    shoot.whileActiveContinuous(new ShootCommand(shooterSub));

    kicker.whenHeld(new KickCommand(kickSub, floorSub));

    reset.whenPressed(new InstantCommand(driveSub::zeroGyroscope, driveSub));

    changeDrive.toggleWhenPressed(
      new RobotDriveCommand(
      () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
      driveSub
    ));
  }
  
  public void resetOdo() {
    driveSub.resetOdometry(path.getInitialPose());
  }


  public Command getSequentialCommand() { 
    Command commandToReturn = new SequentialCommandGroup(
      new SequentialCommandGroup(
      new DriveAuto(driveSub).getPathAuto(0)
     //  driveAuto.getPathAuto(1), 
     //  driveAuto.getPathAuto(2), 
     //  driveAuto.getPathAuto(3)
     ));
      
      return commandToReturn;
   }

  public Command getShootCommand() {
    return new ShootCommand(shooterSub).beforeStarting(new WaitCommand(6));
  }

  public Command getKickerCommand() {
    return new AutoKickerCommand(kickSub);
  }

  public Command getAutoAim() {
    return new AutoAimCommand(hoodSub, turretSub);
  }

  public Command getIntakeCommand() {
    return new AutoIntakeCommand(intakeSub, floorSub);
  }


  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.1) {
        return (value - deadband) / (1.0 - deadband);
      } 
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);
    // value = Math.copySign(value * value, value);
    return value;
  }

  private static boolean getUpDPad() {
    return operator.getPOV() == 0;
  }

  private static boolean getRightDPad() {
    return operator.getPOV() == 90;
  }

  private static boolean getDownDPad() {
    return operator.getPOV() == 180;
  }

  private static boolean getLeftDPad() {
    return operator.getPOV() == 270;
  }

  private static boolean getRightTrigger() {
    return driver.getRightTriggerAxis()>0.05;
  }

  private static boolean getLeftTrigger() {
    return driver.getLeftTriggerAxis()>0.05;
  }

  private static boolean getOperatorLeftBumper() {
    return driver.getRawButton(3);
  }

  private static boolean getOperatorRightBumper() {
    return driver.getRawButton(1);
  }
}