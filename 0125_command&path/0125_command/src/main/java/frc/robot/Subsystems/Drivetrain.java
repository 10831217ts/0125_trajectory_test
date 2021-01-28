// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class Drivetrain extends Subsystem {
  WPI_VictorSPX leftMaster = new WPI_VictorSPX(Constants.leftMaster);
  WPI_VictorSPX rightMaster = new WPI_VictorSPX(Constants.leftSlave);
  WPI_VictorSPX leftSlave = new WPI_VictorSPX(Constants.rightMaster);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(Constants.rightSlave);

  Encoder leftEncoder = new Encoder(2,3, false, EncodingType.k4X);
  Encoder rightEncoder = new Encoder(0,1, false, EncodingType.k4X);
  //DIO pin


  AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //左右輪距離(m)
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  //待解決:不加kinematics?

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243);
  //開控制台 frc-characterization drive new

  PIDController leftPIDController = new PIDController(9.95, 0, 0);
  PIDController rightPIDController = new PIDController(9.95, 0, 0);
  //P要自己找出來

  Pose2d pose;

  public Drivetrain(){
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(gyro.getAngle());
    //待解決:加不加負號
  }

  public DifferentialDriveWheelSpeeds getspeeds() {
    return new DifferentialDriveWheelSpeeds(

      leftEncoder.getRate() * 60/7.29 * 2 * Math.PI * Units.inchesToMeters(3.0) /60,
      rightEncoder.getRate() * 60/7.29 * 2 * Math.PI * Units.inchesToMeters(3.0) /60

      //convert: RPM(rotation per min) of the motor-->(use the gear ratio) RPM of the wheel-->(wheel radius)meters per sec
      //待解決:what's wrrrrrrrrrrong
    );
  }

  

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getrightPIDController(){
    return rightPIDController;
  }

  public Pose2d getPose(){
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts){
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts/12);
  }

  @Override
  public void periodic() {
    //pose = odometry.update(getHeading(), getspeeds());
    pose = odometry.update(getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
}
