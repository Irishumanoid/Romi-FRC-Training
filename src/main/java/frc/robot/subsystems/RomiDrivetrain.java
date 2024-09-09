// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor;
  private final Spark m_rightMotor;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive;

  private final RomiGyro m_gyro;
  private final BuiltInAccelerometer m_accelerometer;
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    m_leftMotor = new Spark(0);
    m_rightMotor = new Spark(1);
    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    m_leftEncoder = new Encoder(4, 5);
    m_rightEncoder = new Encoder(6, 7);
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    m_diffDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

    m_gyro = new RomiGyro();
    m_gyro.reset();
    m_accelerometer = new BuiltInAccelerometer();
    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(getRotX(), getRotY()),
            m_leftEncoder.getDistance(),
            m_rightEncoder.getDistance());
  }

  // In arcade drive, zaxisRotate specifies the speed differential between the two slides of the
  // chassis
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getRotX() {
    return m_gyro.getAngleX();
  }

  public double getRotY() {
    return m_gyro.getAngleY();
  }

  public double getRotZ() {
    return m_gyro.getAngleZ();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getAccelerationX() {
    return m_accelerometer.getX();
  }

  public double getAccelerationY() {
    return m_accelerometer.getY();
  }

  public double getAccelerationZ() {
    return m_accelerometer.getZ();
  }

  @Override
  public void periodic() {
    m_odometry.update(
        new Rotation2d(getRotX(), getRotY()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    SmartDashboard.putNumber(
        "drive/pose-rotation", m_odometry.getPoseMeters().getRotation().getRadians());
    SmartDashboard.putNumber("drive/x-translation", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("drive/y-translation", m_odometry.getPoseMeters().getY());
  }
}
