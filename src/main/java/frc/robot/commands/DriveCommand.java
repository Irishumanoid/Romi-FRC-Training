package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  private final RomiDrivetrain m_drive;
  private final DoubleSupplier speed;
  private final DoubleSupplier rot;
  private final BooleanSupplier isObjectInFOV;

  /**
   * Creates a new ExampleCommand.
   *
   * @param drive The drive used by this command.
   */
  public DriveCommand(
      RomiDrivetrain drive,
      DoubleSupplier speed,
      DoubleSupplier rot,
      BooleanSupplier isObjectInFOV) {
    m_drive = drive;
    this.speed = speed;
    this.rot = rot;
    this.isObjectInFOV = isObjectInFOV;
    // Use addRequirements() here to declare drive dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isObjectInFOV.getAsBoolean()) {
      m_drive.arcadeDrive(0, 0);
    } else {
      m_drive.arcadeDrive(speed.getAsDouble(), rot.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
