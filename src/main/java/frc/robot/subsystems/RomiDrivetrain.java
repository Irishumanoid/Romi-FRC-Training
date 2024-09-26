// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.07;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor;
  private final Spark m_rightMotor;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final PIDController rotController;
  private final PIDController translateController;

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive;

  private final RomiGyro m_gyro;
  private final BuiltInAccelerometer m_accelerometer;
  // private final UltrasonicSensor m_distanceSensor;
  private final DifferentialDriveOdometry m_odometry;
  private final ReplanningConfig replanningConfig = new ReplanningConfig();

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    m_leftMotor = new Spark(0);
    m_rightMotor = new Spark(1);
    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    m_leftEncoder = new Encoder(4, 5);
    m_rightEncoder = new Encoder(6, 7);
    rotController = new PIDController(1, 0, 0.001);
    translateController = new PIDController(1, 0, 0);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetEncoders();

    m_diffDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

    m_gyro = new RomiGyro();
    m_gyro.reset();
    m_accelerometer = new BuiltInAccelerometer();
    // m_distanceSensor = new UltrasonicSensor(3, 4); // TODO set these to the correct channels
    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(m_gyro.getAngle()),
            m_leftEncoder.getDistance(),
            m_rightEncoder.getDistance());

    AutoBuilder.configureLTV(
        this::getPose,
        this::resetOdometry,
        this::getSpeeds,
        this::driveChassisSpeeds,
        0.02,
        replanningConfig,
        this::allianceCheck,
        this);
  }

  // In arcade drive, zaxisRotate specifies the speed differential between the two slides of the
  // chassis
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    arcadeDrive(
        speeds.vxMetersPerSecond / Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        speeds.omegaRadiansPerSecond / Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
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

  public double calculateRotOutput(double curRot, double setpoint) {
    return rotController.calculate(curRot, setpoint);
  }

  public double calculateTranslateOutput(double curDist, double setpoint) {
    return translateController.calculate(curDist, setpoint);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.diffDriveKinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate()));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(m_gyro.getAngle()),
        new DifferentialDriveWheelPositions(
            m_leftEncoder.getDistance(), m_rightEncoder.getDistance()),
        pose);
  }

  private boolean allianceCheck() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  /*public boolean isObjectInFOV() {
    return m_distanceSensor.isObjectTooClose();
  }*/

  @Override
  public void periodic() {
    m_odometry.update(
        new Rotation2d(getRotZ()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    SmartDashboard.putNumber(
        "drive/pose-rotation", m_odometry.getPoseMeters().getRotation().getRadians());
    SmartDashboard.putNumber("drive/x-translation", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("drive/y-translation", m_odometry.getPoseMeters().getY());
  }
}
