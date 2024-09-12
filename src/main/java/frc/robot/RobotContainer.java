// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain;
  private final LEDSubsystem m_leds;
  private final XboxController m_controller = new XboxController(0);

  private final DriveCommand m_autoCommand;
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_romiDrivetrain = new RomiDrivetrain();
    m_autoCommand = new DriveCommand(m_romiDrivetrain, () -> 0.5, () -> 0);
    m_leds = new LEDSubsystem();
    // Configure the button bindings
    configureButtonBindings();
    m_romiDrivetrain.setDefaultCommand(
        new DriveCommand(
            m_romiDrivetrain, () -> -m_controller.getLeftX(), () -> -m_controller.getLeftY()));
    m_leds.setDefaultCommand(
        run(() -> m_leds.setAutoBlinkState(() -> m_controller.getAButton()), m_leds));

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("simple drive", m_autoCommand);
    autoChooser.addOption(
        "multi drive",
        schedulePath(new Command[] {rotateRomi(90, 1), translateRomi(3, 0.5), rotateRomi(180, 0)}));
  }

  public Command rotateRomi(double degrees, double threshold) {
    double setpoint = m_romiDrivetrain.getRotZ() + degrees;
    return run(
            () ->
                m_romiDrivetrain.arcadeDrive(
                    0.1, m_romiDrivetrain.calculateRotOutput(m_romiDrivetrain.getRotZ(), setpoint)),
            m_romiDrivetrain)
        .until(() -> Math.abs(m_romiDrivetrain.getRotZ() - setpoint) < threshold)
        .finallyDo(() -> m_romiDrivetrain.arcadeDrive(0, 0));
  }

  public Command translateRomi(double distInches, double threshold) {
    double setpoint = m_romiDrivetrain.getLeftDistanceInch() + distInches;
    return run(
            () ->
                m_romiDrivetrain.arcadeDrive(
                    m_romiDrivetrain.calculateTranslateOutput(
                        m_romiDrivetrain.getLeftDistanceInch(), setpoint),
                    0),
            m_romiDrivetrain)
        .until(() -> Math.abs(m_romiDrivetrain.getLeftDistanceInch() - setpoint) < threshold)
        .finallyDo(() -> m_romiDrivetrain.arcadeDrive(0, 0));
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
