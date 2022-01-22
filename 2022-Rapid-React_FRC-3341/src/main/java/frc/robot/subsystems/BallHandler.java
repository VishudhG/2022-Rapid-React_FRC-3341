// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;

public class BallHandler extends SubsystemBase {

  //change ports when ready to start testing
  private final WPI_TalonSRX leftflywheel = new WPI_TalonSRX(Constants.MotorPorts.port1);
  private final WPI_TalonSRX rightflywheel = new WPI_TalonSRX(Constants.MotorPorts.port2);
  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.MotorPorts.port3);
  private final WPI_VictorSPX pivotfollower = new WPI_VictorSPX(Constants.MotorPorts.port4);
  private final WPI_VictorSPX roller = new WPI_VictorSPX(Constants.MotorPorts.port5);

  private double ticksToMeters;
  private double ticksToDegrees;


  /** Creates a new ExampleSubsystem. */
  public BallHandler() {
    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    roller.configFactoryDefault();
    roller.setInverted(false);
    pivotfollower.configFactoryDefault();
    pivotfollower.follow(pivot);
    pivotfollower.setInverted(InvertType.FollowMaster);
    leftflywheel.configFactoryDefault();
    leftflywheel.setInverted(false);
    leftflywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightflywheel.configFactoryDefault();
    rightflywheel.setInverted(true);
    rightflywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  

  }

  public void resetEncoders(){
    leftflywheel.setSelectedSensorPosition(0,0,10);
    rightflywheel.setSelectedSensorPosition(0,0,10);
    pivot.setSelectedSensorPosition(0,0,10);
  }

  public double getFlywheelPosition(){
    return (((leftflywheel.getSelectedSensorPosition(0) + rightflywheel.getSelectedSensorPosition(0))/2) * (ticksToMeters));
  }

  public double getPivotPosition(){
    return (((pivot.getSelectedSensorPosition(0) * (ticksToDegrees))));
  }

  public double getVelocity(){
    return (((leftflywheel.getSensorCollection().getPulseWidthVelocity() + rightflywheel.getSensorCollection().getPulseWidthVelocity())/2) * (ticksToMeters));
 }

  public void setFlywheelPower(double speed) {
    leftflywheel.set(speed);
    rightflywheel.set(speed);
  }
  
  public void setPivotPower(double speed) {
    pivot.set(speed);
  }

  public void setVictorPower(WPI_VictorSPX motor, double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
