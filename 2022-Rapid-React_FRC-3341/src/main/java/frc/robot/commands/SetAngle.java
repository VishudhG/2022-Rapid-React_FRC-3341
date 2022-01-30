// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class SetAngle extends CommandBase {
  /** Creates a new SetAngle. */
  private double angle;
  private BallHandler ballhandler;

  public SetAngle(double angle, BallHandler ballHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballHandler);
    this.ballhandler = ballHandler;
    this.angle = angle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballhandler.resetPivotEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballhandler.setPivotPositionPID(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballhandler.setPivotPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(ballhandler.pivotWithinErrorMargin());
  }
}
