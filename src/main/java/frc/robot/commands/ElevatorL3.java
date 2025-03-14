package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class ElevatorL3 extends Command {
    public final ElevatorSubsystem elevatorL3;

    public ElevatorL3(ElevatorSubsystem elevatorCommand) {

        elevatorL3 = elevatorCommand;
        addRequirements(elevatorCommand);

    }

    @Override
    public void initialize() {

        elevatorL3.setPositionInches(Constants.levelThreeHeight);
    }

    @Override
    public void execute() {

        System.out.println("Elevator moving to Level 3...");

    }

    @Override
    public boolean isFinished() {
        return elevatorL3.isAtHeight(Constants.levelThreeHeight);
    }
  
    @Override
    public void end(boolean interrupted) {
      
    }
}