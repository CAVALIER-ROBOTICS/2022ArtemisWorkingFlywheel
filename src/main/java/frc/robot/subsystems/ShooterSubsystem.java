// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
  // CANSparkMax rightShooter = new CANSparkMax(Constants.rightShooterID,MotorType.kBrushless);
  //CANSparkMax leftShooter = new CANSparkMax(Constants.leftShootID,MotorType.kBrushless);

  TalonFX right = new TalonFX(Constants.rightShooterID);
  TalonFX left = new TalonFX(Constants.leftShootID);

  //RelativeEncoder leftEnc = leftShooter.getEncoder();
  // RelativeEncoder rightEnc = rightShooter.getEncoder();
  
  
  //SparkMaxPIDController leftPID = leftShooter.getPIDController();
  // SparkMaxPIDController rightPID = rightShooter.getPIDController();

  


  // leftShooter.enableBrakeMode();
  

  //       talon_.enableLimitSwitch(true, true);
  //       talon_.ConfigFwdLimitSwitchNormallyOpen(true);
  //       talon_.ConfigRevLimitSwitchNormallyOpen(true);
  //       talon_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
  //       talon_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);

  // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.4402, .13453, .0093554);



  public ShooterSubsystem() 
  {
    right.configFactoryDefault();
    left.configFactoryDefault();

    left.setInverted(TalonFXInvertType.Clockwise); //was CounterClockwise - is jittering
    
    right.setInverted(TalonFXInvertType.CounterClockwise); //was Clockwise

    // right.setNeutralMode(NeutralMode.Coast);
    // left.setNeutralMode(NeutralMode.Coast);

    // talon.configClosedloopRamp(0.5);//0.5 seconds from nuetral to full power in closed loop control
    // this means you are using the intergrated sensor using the standard 0 channel(always use 0) and it is updating every 10 ms
    right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
    left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);

    
    // TalonFXConfiguration configs = new TalonFXConfiguration();
    // /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
    // configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    // /* config all the settings */
    // talon.configAllSettings(configs);
    double voltage = 11;

    right.configVoltageCompSaturation(voltage); // "full output" will now scale to 11 Volts for all control modes when enabled.
    right.enableVoltageCompensation(true);

    left.configVoltageCompSaturation(voltage); 
    left.enableVoltageCompensation(true);

    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 65300);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 65310);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 65300);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 65300);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 65300);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 65300);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 65300);
    // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 65300); 

    double ff = 0.068;
    double p = 0.168;//.168
    double i = 0;
    double d = 0;
    right.config_kF(0, ff, 10);
		right.config_kP(0, p, 10);
		right.config_kI(0, i, 10);
		right.config_kD(0, d, 10);

    // left.follow(right);

    left.config_kF(0, ff, 10);
		left.config_kP(0, p, 10);
		left.config_kI(0, i, 10);
		left.config_kD(0, d, 10);




    // rightShooter.restoreFactoryDefaults();
    //leftShooter.restoreFactoryDefaults();
    // rightShooter.setIdleMode(IdleMode.kCoast);
    //leftShooter.setIdleMode(IdleMode.kCoast);
    // rightShooter.setInverted(false);
    //leftShooter.setInverted(true);

    // leftPID.setP(0.00022);
    // leftPID.setI(0.000002);
    // leftPID.setD(0.00009);
    // leftPID.setFF(.000252);

    // rightPID.setP(0.00022);
    // rightPID.setI(0.000002);
    // rightPID.setD(0.00009);
    // rightPID.setFF(.000252);

    // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 64500);
    // rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64400);

    // leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 64300);

  }

  /**
   * 
   * @param a Velocity to be set
   */
  public void setShooterVelocity(double rpm)
  {
    // leftPID.setReference(rpm, ControlType.kVelocity);
    // rightPID.setReference(rpm, ControlType.kVelocity);
    // leftShooter.set(1000, ControlType.kVelocity);
    // leftShooter.set(a);
    // rightShooter.set(a);
    // SmartDashboard.putNumber("Fly Wheel", getVolicty());
    right.set(ControlMode.Velocity, (rpm/600)*2048);
    left.set(ControlMode.Velocity, (rpm/600)*2048);
  }

  public void setShooter(double volt) {
    left.set(ControlMode.PercentOutput, volt);
    right.set(ControlMode.PercentOutput, volt);
  }

  // public double getVolicty()
  // {
  //   // return (leftEnc.getVelocity()+rightEnc.getVelocity())/2;
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler rn
    // SmartDashboard.putNumber("Fly Wheel", getVolicty());
    SmartDashboard.putNumber("fly Wheel", (right.getSelectedSensorVelocity() * 600) / 2048 );
  }
}