// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SoftLimitConfig;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private ShuffleboardTab coralTab = Shuffleboard.getTab("CoralIntake");
  private SparkMax coralIntakeMotor;
  private ProfiledPIDController pid;
  private AbsoluteEncoder absEncoder;
  private double targetPosition = 0;

  public CoralIntake() {
    SparkMax coralIntakeMotor = new SparkMax(13, MotorType.kBrushless);
    

    pid = new ProfiledPIDController(0.1, 0,0, new Constraints(0.5, 2));
    // pid.enableContinuousInput(0, 1);

    setTargetPosition(targetPosition);

    absEncoder = coralIntakeMotor.getAbsoluteEncoder();

    coralTab.addNumber("Position", () -> getPosition());
    // setSoftLimit();
  }
  
//    public void setSoftLimit(){
//     coralIntakeMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
//     coralIntakeMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

//    }

  public void moveArm(double power) {
    coralIntakeMotor.set(power);
  }


  public double getPosition() {
    return absEncoder.getPosition();
  }

  public void setTargetPosition(double position){
    targetPosition = position;
    pid.setGoal(targetPosition);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  @Override
  public void periodic() {
    
    coralIntakeMotor.set(pid.calculate(getPosition()));
  
  }
}