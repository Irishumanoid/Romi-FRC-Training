package frc.robot.subsystems.Drive;

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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Drivetrain extends SubsystemBase {

  private final DifferentialDrive m_diffDrive;
  private final Wheel m_leftWheel;
  private final Wheel m_rightWheel;

  private final RomiGyro m_gyro;
  private final BuiltInAccelerometer m_accelerometer;
  private final DifferentialDriveOdometry m_odometry; // TODO use pose estimator to track pose
  private final ReplanningConfig replanningConfig = new ReplanningConfig();

  private final PIDController rotController;
  private final PIDController translateController;

  public Drivetrain(WheelIO left, WheelIO right) {
    m_leftWheel = new Wheel(left, 0);
    m_rightWheel = new Wheel(right, 1);
    m_diffDrive = new DifferentialDrive(m_leftWheel::set, m_rightWheel::set);

    m_gyro = new RomiGyro();
    m_gyro.reset();
    m_accelerometer = new BuiltInAccelerometer();

    m_odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(m_gyro.getAngle()),
            m_leftWheel.getPosition(),
            m_rightWheel.getPosition());

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        translateController = new PIDController(0.05, 0.0, 0.0);
        rotController = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        translateController = new PIDController(0.1, 0.0, 0.0);
        rotController = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        translateController = new PIDController(0.1, 0.0, 0.0);
        rotController = new PIDController(10.0, 0.0, 0.0);
        break;
    }

    AutoBuilder.configureRamsete(
        this::getPose,
        this::resetOdometry,
        this::getSpeeds,
        this::driveChassisSpeeds,
        replanningConfig,
        this::allianceCheck,
        this);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    System.out.println("speed" + xaxisSpeed);
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    arcadeDrive(
        speeds.vxMetersPerSecond / Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        speeds.omegaRadiansPerSecond / Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
  }

  public void resetEncoders() {
    m_leftWheel.resetEncoder();
    m_rightWheel.resetEncoder();
  }

  public double getLeftDistanceMeter() {
    return m_leftWheel.getPosition();
  }

  public double getRightDistanceMeter() {
    return m_rightWheel.getPosition();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getAngle() {
    return m_gyro.getAngle() % 360;
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

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double calculateRotOutput(double curRot, double setpoint) {
    return rotController.calculate(curRot, setpoint);
  }

  public double calculateTranslateOutput(double curDist, double setpoint) {
    return translateController.calculate(curDist, setpoint);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.diffDriveKinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(m_leftWheel.getVelocity(), m_rightWheel.getVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(m_gyro.getAngle()),
        new DifferentialDriveWheelPositions(m_leftWheel.getPosition(), m_rightWheel.getPosition()),
        pose);
  }

  private boolean allianceCheck() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  @Override
  public void periodic() {
    m_leftWheel.periodic();
    m_rightWheel.periodic();
    m_odometry.update(
        new Rotation2d(m_gyro.getAngle()), m_leftWheel.getPosition(), m_rightWheel.getPosition());
    SmartDashboard.putNumber(
        "drive/pose-rotation", m_odometry.getPoseMeters().getRotation().getRadians());
    SmartDashboard.putNumber("drive/x-translation", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("drive/y-translation", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("drive/angle", getAngle());
  }
}
