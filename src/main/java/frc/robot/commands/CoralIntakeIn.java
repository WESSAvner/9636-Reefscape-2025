package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;


public class CoralIntakeIn extends Command {
    public final CoralSubsystem intakeIn;

    public CoralIntakeIn(CoralSubsystem intakeCommand) {
        
        intakeIn = intakeCommand;
        addRequirements(intakeCommand);

    }

    @Override
    public void initialize() {
        intakeIn.coralIntakeIn();
        
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