// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/**
 * This is a sample program that uses mecanum drive with a gyro sensor to maintain rotation vectors
 * in relation to the starting orientation of the robot (field-oriented controls).
 */
public class Robot extends TimedRobot {
  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  private static final int kFrontLeftChannel = 0;
  private static final int kRearLeftChannel = 1;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  private static final int kGyroPort = 0;
  private static final int kJoystickPort = 0;
  private static final boolean boolBreakMode = false;

  private final MecanumDrive m_robotDrive;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);
  private final XboxController operatorController = new XboxController(constants.operatorStick);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushless);
    SparkMaxConfig config1 = new SparkMaxConfig();
    SparkMaxConfig config2 = new SparkMaxConfig();


    // Inverted configuration for the motors. Change the boolean of kbreakMode to true to enable/disable break mode.
    // Congig1 is for the rightside motors, as they are inverted, config2 is for the leftside motors.
    if (boolBreakMode) {
      config1.inverted(true).idleMode(IdleMode.kBrake);
      config2.inverted(false).idleMode(IdleMode.kBrake);
    } else if (!boolBreakMode) {
      config1.inverted(true).idleMode(IdleMode.kCoast);
      config2.inverted(false).idleMode(IdleMode.kCoast);
    }

    // Invert the right side motors.
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

  // public static void updateControls() {
  //   // Call this method in teleopPeriodic() to upstate controls. 
  //   // The controls are renamed via this method for ease of reading, but consequently must be updated constantly.

  //   double 

  // }

  /** Mecanum drive is used with the gyro angle as an input. */
  @Override
  public void teleopPeriodic() {
    // m_robotDrive.driveCartesian(
    //     -m_joystick.getY(), -m_joystick.getX(), -m_joystick.getZ(), m_gyro.getRotation2d());
    m_robotDrive.driveCartesian(
      -m_joystick.getY(), -m_joystick.getX(), -m_joystick.getZ());
  }
}
