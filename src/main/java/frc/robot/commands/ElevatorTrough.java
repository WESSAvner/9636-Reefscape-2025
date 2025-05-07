package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorTrough extends Command {
    public final ElevatorSubsystem elevatorTrough;

    public ElevatorTrough(ElevatorSubsystem elevatorCommand) {

        elevatorTrough = elevatorCommand;
        addRequirements(elevatorCommand);

    }

    @Override
    public void initialize() {

        elevatorTrough.setPositionInches(Constants.levelOneHeight);
        System.out.println("Elevator moving to Level One...");

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return elevatorTrough.isAtHeight(Constants.levelOneHeight);
    }
  
    @Override
    public void end(boolean interrupted) {
      
    }
}