// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler;

public class ManualRoller extends CommandBase {

  
  private BallHandler ballhandler;
  private double power;
  
  /** Creates a new ManualRoller. */
  public ManualRoller(double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnBallHandler());
    this.ballhandler = RobotContainer.returnBallHandler();
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballhandler.setRollerPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballhandler.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
