package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

// This is the same as "CoralIntakeIn.java" but its for the other direction.
public class CoralIntakeOut extends Command {
    public final CoralSubsystem intakeOut;

    public CoralIntakeOut(CoralSubsystem intakeCommand) {
        
        intakeOut = intakeCommand;
        addRequirements(intakeCommand);

    }

    @Override
    public void initialize() {
        intakeOut.coralOutput();
        
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
        intakeOut.coralStop();
    }
}