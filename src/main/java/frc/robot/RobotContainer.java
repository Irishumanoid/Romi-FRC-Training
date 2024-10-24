// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.subsystems.Drive.WheelIO;
import frc.robot.subsystems.Drive.WheelIOSim;
import frc.robot.subsystems.Drive.WheelIOSpark;
import frc.robot.subsystems.LEDSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain;
  private final LEDSubsystem m_leds;
  private final Joystick m_controller = new Joystick(0);

  private final DriveCommand m_autoCommand;
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        m_drivetrain =
            new Drivetrain(new WheelIOSpark(0, 4, 5, false), new WheelIOSpark(1, 6, 7, true));
        break;
      case SIM:
        m_drivetrain = new Drivetrain(new WheelIOSim(), new WheelIOSim());
        break;
      default:
        m_drivetrain = new Drivetrain(new WheelIO() {}, new WheelIO() {});
    }

    m_autoCommand = new DriveCommand(m_drivetrain, () -> 0.5, () -> 0, () -> false);
    m_leds = new LEDSubsystem();

    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(
        new DriveCommand(
            m_drivetrain, () -> -m_controller.getX(), () -> -m_controller.getY(), () -> false));
    m_leds.setDefaultCommand(run(() -> m_leds.setAutoBlinkState(() -> true), m_leds));
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("simple drive", m_autoCommand);
    autoChooser.addOption(
        "multi drive", schedulePath(new Command[] {translateRomi(0.5, 0.5), rotateRomi(180, 10)}));
    TurnDegrees turnCommand = new TurnDegrees(0.9, 180, m_drivetrain);
    autoChooser.addOption("rotate romi command", turnCommand);
    SmartDashboard.putData("autoChooser", autoChooser);
    autoChooser.addOption("path planner", AutoBuilder.buildAuto("testauto"));
  }

  public Command rotateRomi(double degrees, double threshold) {
    double initSetpoint = m_drivetrain.getAngle() + degrees;
    if (Math.signum(m_drivetrain.getAngle()) != Math.signum(initSetpoint)) {
      initSetpoint += -Math.signum(initSetpoint) * 360;
    }
    double setpoint = initSetpoint;

    return run(
            () -> {
              double factor = Math.signum(setpoint) * Math.signum(m_drivetrain.getAngle());
              double output =
                  m_drivetrain.calculateRotOutput(m_drivetrain.getAngle() * factor, setpoint);
              SmartDashboard.putNumber("rotPidOutput", output);
              m_drivetrain.arcadeDrive(0, output);
            },
            m_drivetrain)
        .until(
            () ->
                Math.abs(
                        Math.signum(setpoint)
                                * Math.signum(m_drivetrain.getAngle())
                                * m_drivetrain.getAngle()
                            - setpoint)
                    < threshold)
        .finallyDo(() -> m_drivetrain.arcadeDrive(0, 0));
  }

  public Command translateRomi(double distMeters, double threshold) {
    double setpoint = m_drivetrain.getLeftDistanceMeter() + distMeters;
    return run(
            () -> {
              double output =
                  m_drivetrain.calculateTranslateOutput(
                      m_drivetrain.getLeftDistanceMeter(), setpoint);
              SmartDashboard.putNumber("translationPidOutput", output);
              m_drivetrain.arcadeDrive(output, 0);
            },
            m_drivetrain)
        .until(() -> Math.abs(m_drivetrain.getLeftDistanceMeter() - setpoint) < threshold)
        .finallyDo(() -> m_drivetrain.arcadeDrive(0, 0));
  }

  public Command schedulePath(Command[] pathCommands) {
    return sequence(pathCommands);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
