// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.BallHandler;


public class EncoderShoot extends CommandBase {

  BallHandler ballhandler;
  private Joystick joystick;

  double power;
  double desiredvelocity;
  boolean isFlywheelAtSpeed;
  private boolean teleOp;
  //placeholder
  double cargoIsLaunchedTime = 0;

  Timer cargoTimer = new Timer();

  //placeholder
  private double rollerpower = 0;

  /** Creates a new EncoderShoot. */
  public EncoderShoot(BallHandler ballHandler, double power, Joystick joystick, boolean teleOp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballHandler);
    this.ballhandler = ballHandler;
    this.power = power;
    this.joystick = joystick;
    this.teleOp = teleOp;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFlywheelAtSpeed = false;
    cargoTimer.reset();
    ballhandler.resetFlywheelEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(teleOp){
      ballhandler.setFlywheelPower(joystick.getRawAxis(1));
    }
    else {
    ballhandler.setFlywheelConstantVelocity(power);

    if(ballhandler.flywheelWithinErrorMargin() && !isFlywheelAtSpeed) {
      ballhandler.setRollerPower(rollerpower);
      cargoTimer.start();
      isFlywheelAtSpeed = true;
    }
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballhandler.setFlywheelPower(0);
    ballhandler.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (cargoTimer.get() >= cargoIsLaunchedTime);
    
  }
}
