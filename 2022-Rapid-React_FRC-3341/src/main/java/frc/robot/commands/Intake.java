
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.InfraredSensor;

public class Intake extends CommandBase {
  private BallHandler ballHandler = new BallHandler();
  private InfraredSensor infrared = new InfraredSensor();
  /** Creates a new Intake. */
  public Intake(BallHandler b, InfraredSensor ir) {
    ballHandler = b;
    infrared = ir;
    addRequirements(ballHandler, infrared);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ballHandler.isForwardLimitClosed()){
      ballHandler.setPivotPower(-0.1); //motor goes clockwise
    } else {
      ballHandler.setPivotPower(0.0);
        ballHandler.setFlywheelPower(0.1);
        ballHandler.setRollerPower(-0.1); //roller goes clockwise
      } 
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballHandler.setFlywheelPower(0);
    ballHandler.setRollerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return infrared.get();
  }
}