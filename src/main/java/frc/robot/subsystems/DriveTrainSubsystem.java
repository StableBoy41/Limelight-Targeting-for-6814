// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private static final CANSparkMax left1 = new CANSparkMax(15, MotorType.kBrushed);
  private static final CANSparkMax left2 = new CANSparkMax(9, MotorType.kBrushed);
  private static final CANSparkMax right1 = new CANSparkMax(11, MotorType.kBrushed);
  private static final CANSparkMax right2 = new CANSparkMax(13, MotorType.kBrushed);
  
  private final Encoder lEncoder = new Encoder(0, 1);
  private final Encoder rEncoder = new Encoder(4, 5);
  
  private static final MotorControllerGroup m_left = new MotorControllerGroup(left1, left2);
  private static final MotorControllerGroup m_right = new MotorControllerGroup(right1, right2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void SetMotors(double leftSpeed, double rightSpeed) {
    m_left.set(leftSpeed);
    m_right.set(-rightSpeed);
  }

  public void noSafe() {
    m_robotDrive.setSafetyEnabled(false);
  }

  public Encoder getLeftEncoder() {
    return lEncoder;
  }

  public void resetEncoders(){
    lEncoder.reset();
    rEncoder.reset();
  }
}