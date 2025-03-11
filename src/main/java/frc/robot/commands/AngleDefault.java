
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralIntake;
// import frc.robot.Constants;

// public class AngleDefault extends Command {
//   private CoralIntake angleDefault;

//   /** Creates a new MoveArmToPostion. */
//   public AngleDefault(CoralIntake angleCommand) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     angleDefault = angleCommand;

//     addRequirements(angleCommand);
    
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // angleDefault.setTargetPosition(Constants.coralDefaultAngle);
//     angleDefault.setAngle(Constants.coralDefaultAngle);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // angleDefault.coralIntakeMotor.set(angleDefault.pid.calculate(angleDefault.getPosition(), angleDefault.getTargetPosition()));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }