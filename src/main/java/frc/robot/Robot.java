// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.joystickScale;


/**
 * This is a sample program that uses mecanum drive with a gyro sensor to maintain rotation vectors
 * in relation to the starting orientation of the robot (field-oriented controls).
 */
public class Robot extends TimedRobot {
  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 4;

  private static final int kclawChannel = 6;
  private static final int kwristChannel = 7;
  private static final int kelevatorChannel = 5;

  private static final int kGyroPort = 0;
  private static final int kDriverPort = 0;
  public static final int kOperatorPort = 1;
  private static final boolean boolBreakMode = false;

  private final MecanumDrive m_robotDrive;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kDriverPort);
  private final XboxController operatorController = new XboxController(kOperatorPort);

  private final SparkMax elevatorSpark = new SparkMax(kelevatorChannel, MotorType.kBrushless);
  private final SparkMax clawSpark = new SparkMax(kclawChannel, MotorType.kBrushless);
  private final WPI_VictorSPX wristVictor = new WPI_VictorSPX(kwristChannel);
  private final SparkMaxConfig clawConfigLowAmp = new SparkMaxConfig();
  private final SparkMaxConfig clawConfigHighAmp = new SparkMaxConfig();
  

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SmartDashboard.putNumber("check", kDriverPort);

    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushless);
    

    SparkMaxConfig config1 = new SparkMaxConfig();
    SparkMaxConfig config2 = new SparkMaxConfig();
    SparkMaxConfig clawConfig = new SparkMaxConfig();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    clawConfigHighAmp.inverted(true);
    clawConfigLowAmp.inverted(true);

    // Set wrist inverted
   // wristVictor.setInverted(false)

    // Inverted configuration for the motors. Change the boolean of kbreakMode to true to enable/disable break mode.
    // Congig1 is for the rightside motors, as they are inverted, config2 is for the leftside motors.
    if (boolBreakMode) {
      config1.inverted(true).idleMode(IdleMode.kBrake);
      config2.inverted(false).idleMode(IdleMode.kBrake);
    } else if (!boolBreakMode) {
      config1.inverted(true).idleMode(IdleMode.kCoast);
      config2.inverted(false).idleMode(IdleMode.kCoast);
    }

    // Configure the claw motor.
    // Stall limit current determines how hard the motor tries to turn before stalling/giving up. 
    // Make sure this is not too high so that the motor won't burn out, and not too low that it won't work.
    clawConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(8);
    clawSpark.configure(clawConfig, null, null);

    // Configure the elevator motor.
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorSpark.configure(elevatorConfig, null, null);

    // Invert the right side drive motors.
    // You may need to change or remove this to match your robot.
    frontRight.configure(config1, null, null);
    rearRight.configure(config1, null, null);

    // Do not invert the leftside motors, but configure to match the brakeMode.
    frontLeft.configure(config2, null, null);
    rearLeft.configure(config2, null, null);

    m_robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);

    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);

    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);

  }

  // Speical thanks to Robert! This is the auto part:
  SequentialCommandGroup moveForward;

  @Override
  public void autonomousInit() {
    long t= System.currentTimeMillis();
    long end = t+5000;
    while(System.currentTimeMillis() < end) {
       moveForward = new SequentialCommandGroup(
      new ParallelRaceGroup(new InstantCommand(() -> m_robotDrive.driveCartesian(0.35, 0, 0)),
      new WaitCommand(10)),
      new InstantCommand(() -> m_robotDrive.driveCartesian(0, 0, 0))
    );
    moveForward.schedule();

    }
    
  }


  @Override
  public void teleopInit() {
    if (moveForward != null)
      moveForward.cancel();
  }

  // Maybe a function to changes sensitivity for the operator?

  /** Mecanum drive is used with the gyro angle as an input. */
  @Override
  public void teleopPeriodic() {
    double joystickSpeedY;
    double joystickSpeedX;
    double joystickSpeedZ;

    double elevatorSpeed = -MathUtil.applyDeadband(operatorController.getLeftY(), constants.operatorDeadband);
    double wristSpeed = MathUtil.applyDeadband(operatorController.getRightY(), constants.operatorDeadband) * 0.5; // 50% speed
    double clawOpen = MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), constants.operatorDeadband);
    double clawClose = MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), constants.operatorDeadband);
    // m_robotDrive.driveCartesian(
    //     -m_joystick.getY(), -m_joystick.getX(), -m_joystick.getZ(), m_gyro.getRotation2d());

    m_robotDrive.driveCartesian(
      -MathUtil.applyDeadband(m_joystick.getY(), constants.driverDeadband), 
      MathUtil.applyDeadband(m_joystick.getX(), constants.driverDeadband), 
      MathUtil.applyDeadband(m_joystick.getZ(), constants.driverDeadband));

  //  if (m_joystick.getRawButton(1)) {
  //   m_robotDrive.driveCartesian(
  //     -joystickScale.ScaleJoystick(m_joystick.getY(), 0.0, 1.35) * 0.35, 
  //     -joystickScale.ScaleJoystick(m_joystick.getX(), 0.0, 1.35) * 0.35, 
  //     -joystickScale.ScaleJoystick(m_joystick.getZ(), 0.0, 1.35) * 0.35);
  //  } else { 
  //   m_robotDrive.driveCartesian(
  //     -joystickScale.ScaleJoystick(m_joystick.getY(), 0.0, 1.75), 
  //     -joystickScale.ScaleJoystick(m_joystick.getX(), 0.0, 1.75), 
  //     -joystickScale.ScaleJoystick(m_joystick.getZ(), 0.0, 1.75));
  //  }
    // This is the elevator control section:
    {
      if (elevatorSpeed > 0) {
        elevatorSpeed = elevatorSpeed;
        // elevatorSpeed = elevatorSpeed * constants.elExtensionSpeed;
      } else if (elevatorSpeed < 0) {
        elevatorSpeed = -constants.elRetractionSpeed;
      } else {
        elevatorSpeed = 0;
      }
      
      SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);
      elevatorSpark.set(elevatorSpeed);
    }
    



    // This is the wrist control section:
    // This hopefully gives the operator more control when the wrist is hanging out. it will likely tend to
    // dip down when do driven, so if the operator gives minimal input, it will use the steady speed found in constanst file.
    // *this will likely need to be adjuested. may set it somewhere between what is needed for the coral vs the ball.
    {
      double motorSpeed;
      wristSpeed = wristSpeed * 2;
      if (wristSpeed >= 0.65) {
        motorSpeed = constants.wristRegSpeed;
      } else if (wristSpeed < 0.65 && wristSpeed > 0) {
        motorSpeed = constants.wristSteadySpeed;
      } else if (wristSpeed <= -0.65) {
        motorSpeed = -constants.wristRegSpeed;
      } else if (wristSpeed > -0.65 && wristSpeed < 0) {
        motorSpeed = -constants.wristSteadySpeed;
      } else {
        motorSpeed = 0;
      }
      SmartDashboard.putNumber("Wrist Input", wristSpeed);
      SmartDashboard.putNumber("Wrist Speed", motorSpeed);
      wristVictor.set(motorSpeed);
    }




    // This is the claw control section:
    {
      double triggerMinimum = 0.5; // This is the minimum value for the triggers to be considered pressed.
      double clawAction;
      if (clawOpen > triggerMinimum && clawClose < triggerMinimum) {
        clawAction = constants.clawSpeed;
      } else if (clawClose > triggerMinimum && clawOpen < triggerMinimum) {
        clawAction = constants.clawSpeedReverse;
      } else {
        clawAction = 0;
      }

      SmartDashboard.putNumber("Claw Speed", clawAction);
      clawSpark.set(clawAction);
    }

  }

  
}
