
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralIntake;
// import frc.robot.Constants;

// public class AngleSet50 extends Command {
//   private CoralIntake angleSet50;

//   /** Creates a new MoveArmToPostion. */
//   public AngleSet50(CoralIntake angleCommand) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     angleSet50 = angleCommand;
    
//     addRequirements(angleCommand);

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // angleSet50.setTargetPosition(Constants.coralAngle50);
//     angleSet50.setAngle(Constants.coralAngle50);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // angleSet50.coralIntakeMotor.set(angleSet50.pid.calculate(angleSet50.getPosition(), angleSet50.getTargetPosition()));

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