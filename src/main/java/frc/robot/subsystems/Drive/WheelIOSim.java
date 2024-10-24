package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class WheelIOSim implements WheelIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private double driveAppliedVolts = 0.0;

  @Override
  public void updateInputs(WheelIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionMeters =
        driveSim.getAngularPositionRotations() * Math.PI * Constants.kWheelDiameterMeters;
    inputs.driveVelocityMetersPerSec =
        driveSim.getAngularVelocityRPM() * Math.PI * Constants.kWheelDiameterMeters / 60;
    inputs.driveAppliedVolts = driveAppliedVolts;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setDriveSpeed(double speed) {
    double scale = MathUtil.clamp(speed, -1.0, 1.0);
    setDriveVoltage(scale * 12);
  }
}
