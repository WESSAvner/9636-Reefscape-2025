package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

// fuck elevators, all my homies hate elevators.
public class ElevatorL2 extends Command {
    public final ElevatorSubsystem elevatorL2;

    public ElevatorL2(ElevatorSubsystem elevatorCommand) {

        elevatorL2 = elevatorCommand;
        addRequirements(elevatorCommand);
    }

    @Override
    public void initialize() {
        // At the beginning we set te height to whatever height we have in the Constants.java file
        elevatorL2.setPositionInches(Constants.levelTwoHeight);
        System.out.println("Elevator moving to Level 2...");

    }

    @Override
    public void execute() {


    }

    @Override
    public boolean isFinished() {
        return elevatorL2.isAtHeight(Constants.levelTwoHeight);
        //here, we dont have "return false;" but instead wether or not the elevator is at the height.
    }
  
    @Override
    public void end(boolean interrupted) {
      
    }
}