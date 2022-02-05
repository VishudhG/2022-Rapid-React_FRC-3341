package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class InfraredSensor extends SubsystemBase { 
  /** Creates a new InfraredSensor */

  DigitalInput InfraredInput;

  public InfraredSensor() {
    InfraredInput = new DigitalInput(0);
  }

  public boolean get(){
    return !(InfraredInput.get());
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ball Intaken", InfraredInput.get());
    // This method will be called once per scheduler run
  }
}