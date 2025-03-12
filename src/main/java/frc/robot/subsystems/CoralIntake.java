// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.AbsoluteEncoderConfig;
// import com.revrobotics.spark.config.SoftLimitConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SoftLimitConfig;


// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class CoralIntake extends SubsystemBase {
//   public ShuffleboardTab coralTab = Shuffleboard.getTab("CoralIntake");
//   public SparkMax coralIntakeMotor = new SparkMax(7, MotorType.kBrushless);
//   public ProfiledPIDController pid = new ProfiledPIDController(0.001, 0,0, new Constraints(0.1, 0.1));

//   public AbsoluteEncoder absEncoder = coralIntakeMotor.getAbsoluteEncoder();
// //   public double targetPosition;

//   public CoralIntake() {
//     //SparkMax coralIntakeMotor = new SparkMax(7, MotorType.kBrushless);
    

//     // pid = new ProfiledPIDController(0.1, 0,0, new Constraints(0.5, 2));
//     // fact reset, off continuous, add to zero, 
//     // pid.enableContinuousInput(0, 360);


//     absEncoder = coralIntakeMotor.getAbsoluteEncoder();
//     // AbsoluteEncoderConfig.zeroOffset(0);

    
//     coralTab.addNumber("Position", () -> getPosition());
//     coralTab.addNumber("Target Position", () -> getTargetPosition());
//     coralTab.addNumber("PID Output", () -> Math.round(pid.calculate(getPosition(), getTargetPosition()) * 10000d) / 10000d);
//     // setSoftLimit();
//   }

//   public void zeroEncoder(){

//     SparkMaxConfig config = new SparkMaxConfig();
//     config.absoluteEncoder.zeroOffset(getPosition());

//     coralIntakeMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

//   }
  
// //    public void setSoftLimit(){
// //     coralIntakeMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
// //     coralIntakeMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

// //    }

  

//   public void setAngle(double coralAngle) {

//     setTargetPosition(coralAngle);
    
//     coralIntakeMotor.set(pid.calculate(getPosition(), getTargetPosition()));

//   }

//   public void moveArm(double power) {
//     coralIntakeMotor.set(power);
//   }


//   public double getPosition() {
//     return absEncoder.getPosition();
    
//   }

//   public void setTargetPosition(double position){
//     pid.setGoal(position);
//   }

//   public double getTargetPosition() {
//     // return targetPosition;
//     return pid.getGoal().position;
//   }

//   @Override
//   public void periodic() {
    
//     // coralIntakeMotor.set(pid.callculate(getPosition(), getTargetPosition()));
  
//   }
// }