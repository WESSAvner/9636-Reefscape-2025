package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorResting extends Command {
    public final ElevatorSubsystem elevatorResting;

    public ElevatorResting(ElevatorSubsystem elevatorCommand) {

        elevatorResting = elevatorCommand;
        addRequirements(elevatorCommand);

    }

    @Override
    public void initialize() {

        elevatorResting.setPositionInches(Constants.restingHeight);
        System.out.println("Elevator moving to Base Height...");

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return elevatorResting.isAtHeight(Constants.restingHeight);
    }
  
    @Override
    public void end(boolean interrupted) {
      
    }
}