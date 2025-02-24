package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class ElevatorL3 extends Command {
    public final ElevatorSubsystem elevator;

    public ElevatorL3(ElevatorSubsystem elevatorCommand) {

        elevator = elevatorCommand;

    }

    @Override
    public void initialize() {

        elevator.setPositionInches(Constants.levelThreeHeight);
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