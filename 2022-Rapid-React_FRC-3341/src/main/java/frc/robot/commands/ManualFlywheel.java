// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;

public class ManualFlywheel extends CommandBase {

  private BallHandler ballhandler;
  private double velocity;
  
  
  /** Creates a new ManualFlywheel. */
  public ManualFlywheel(double velocity) {

    addRequirements(RobotContainer.returnBallHandler());
    this.ballhandler = RobotContainer.returnBallHandler();
    this.velocity = velocity;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballhandler.resetFlywheelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballhandler.setFlywheelConstantVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballhandler.setFlywheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
