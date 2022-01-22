// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class BallHandler extends SubsystemBase {

  private final WPI_TalonSRX motor1 = new WPI_TalonSRX(Constants.MotorPorts.port1);
  private final WPI_TalonSRX motor2 = new WPI_TalonSRX(Constants.MotorPorts.port1);
  private final WPI_TalonSRX motor3 = new WPI_TalonSRX(Constants.MotorPorts.port1);
  private final WPI_TalonSRX motor4 = new WPI_TalonSRX(Constants.MotorPorts.port1);
  private final Servo servo = new Servo(Constants.MotorPorts.port2);


  /** Creates a new ExampleSubsystem. */
  public BallHandler() {
      motor1.configFactoryDefault();
      motor1.setInverted(false);
      motor2.configFactoryDefault();
      motor2.setInverted(false);
      motor3.configFactoryDefault();
      motor3.setInverted(false);
      motor4.configFactoryDefault();
      motor4.setInverted(false);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
