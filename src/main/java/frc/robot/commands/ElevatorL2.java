package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;


public class ElevatorL2 extends Command {
    public final ElevatorSubsystem elevatorL2;

    public ElevatorL2(ElevatorSubsystem elevatorCommand) {

        elevatorL2 = elevatorCommand;

    }

    @Override
    public void initialize() {

        elevatorL2.setPositionInches(Constants.levelTwoHeight);
    }

    @Override
    public void execute() {


    }

    @Override
    public boolean isFinished() {
      
      return false;
    }
  
    @Override
    public void end(boolean interrupted) {
      
    }
}