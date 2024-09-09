// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveDirections;
import frc.robot.commands.DriveCommand;
import frc.robot.lib.Tuple;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.List;

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
  }

  /**
   * @param simplePath list of pairs of drive directions and distances (inches) in the respective
   *     directions
   */
  public void schedulePath(List<Tuple<DriveDirections, Double>> simplePath) {
    sequence(print("unimplemented")); // should be a sequence of rotation and translation commands
    // run(() -> m_romiDrivetrain.arcadeDrive(1, 0), m_romiDrivetrain).until(() ->
    // m_romiDrivetrain.getLeftDistanceInch() > 4);
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
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
