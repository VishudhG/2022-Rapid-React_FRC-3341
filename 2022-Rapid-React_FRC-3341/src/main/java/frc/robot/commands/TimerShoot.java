// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class TimerShoot extends CommandBase {

  private double power;
  //placeholder
  private double rollerpower = 0;

  boolean isFlywheelAtSpeed;
  //placeholders are 0s
  double flyWheelUpToSpeedTime = 0;
  double cargoIsLaunchedTime = 0;
  Timer flywheelTimer;
  Timer cargoTimer;

  BallHandler ballhandler;

  /** Creates a new Shoot. */
  public TimerShoot(BallHandler ballHandler, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballHandler);
    this.ballhandler = ballHandler;
    this.power = power;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isFlywheelAtSpeed = false;
    flywheelTimer = new Timer();
    flywheelTimer.reset();
    cargoTimer = new Timer();
    cargoTimer.reset();

    flywheelTimer.start();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ballhandler.setFlywheelPower(power);

    if(flywheelTimer.get() >= flyWheelUpToSpeedTime && !isFlywheelAtSpeed) {
        ballhandler.setRollerPower(rollerpower);
        flywheelTimer.reset();
        cargoTimer.start();
        isFlywheelAtSpeed = true;
    }
      
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballhandler.setRollerPower(0);
    ballhandler.setFlywheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cargoTimer.get() >= cargoIsLaunchedTime) {
      return true;
    }
    else return false;
  }
}
