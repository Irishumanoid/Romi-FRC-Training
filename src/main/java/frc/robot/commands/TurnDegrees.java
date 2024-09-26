package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;

public class TurnDegrees extends Command {
  private final RomiDrivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;


  public TurnDegrees(double speed, double degrees, RomiDrivetrain drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }


  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  @Override
  public void execute() {
    System.out.printf("executing with angle %f", m_drive.getPose().getRotation().getDegrees());
    System.out.printf("commanded turn speed is %f", m_speed);
    m_drive.arcadeDrive(0.1, m_speed);
  }


  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }


  @Override
  public boolean isFinished() {
    // diameter/degrees per rot
    double meterPerDegree = Math.PI * 0.141 / 360;
    return getAverageTurningDistance() >= (meterPerDegree * m_degrees);
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceMeter());
    double rightDistance = Math.abs(m_drive.getRightDistanceMeter());
    return (leftDistance + rightDistance) / 2.0;
  }
}