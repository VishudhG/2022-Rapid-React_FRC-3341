// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static Joystick joystick;
  private static JoystickButton shootbutton;
  private static JoystickButton flywheelbutton;
  private static JoystickButton rollerbutton;
  private static JoystickButton setanglebutton;
  private static JoystickButton resetanglebutton;

  private int flywheelpower = 0;
  private double flywheelvelocity = 0;

  private double angle = 0;

  private int rollerpower = 0;

  private static final BallHandler m_ballHandler = new BallHandler();
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    joystick = new Joystick(Constants.JoystickPorts.JoystickPort1);
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    shootbutton = new JoystickButton(joystick, 1);
    shootbutton.toggleWhenPressed(new EncoderShoot(flywheelpower));
    flywheelbutton = new JoystickButton(joystick, 2);
    flywheelbutton.toggleWhenPressed(new ManualFlywheel(flywheelvelocity));
    rollerbutton = new JoystickButton(joystick, 3);
    rollerbutton.toggleWhenPressed(new ManualRoller(rollerpower));
    setanglebutton = new JoystickButton(joystick, 4);
    setanglebutton.toggleWhenPressed(new SetAngle(angle));
    resetanglebutton = new JoystickButton(joystick, 5);
    resetanglebutton.toggleWhenPressed(new SetAngle(0));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public static BallHandler returnBallHandler() {
    return m_ballHandler;
  }
}
