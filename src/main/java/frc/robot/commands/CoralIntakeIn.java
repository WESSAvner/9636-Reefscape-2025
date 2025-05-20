package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* 
this is the beginning of the COMMANDS wow so cool
if you want to write a command, just replace the variables that make it up.
for example, replace "CoralSubsystem" with the name of the subsystem that is being moved.
if you hover over anything or press Command+Left Click, you can see where any term is called anywhere in the project.
*/
public class CoralIntakeIn extends Command {
    public final CoralSubsystem intakeIn;

    // This defines what subsystem is being changed or moved.
    public CoralIntakeIn(CoralSubsystem intakeCommand) {
        
        intakeIn = intakeCommand;
        addRequirements(intakeCommand);

    }

    // Hover our cursor over "initialize" to see what it does. do the same for "execute" "isFinished" and "end"
    // This says that whatever is in "initialize" will happen ONCE at the BEGINNING of the command when its called.
    @Override
    public void initialize() {

        // This version of "coralIntakeIn" is the actual method within the "CoralSubsystem.java" file.
        intakeIn.coralIntakeIn();
        
    }

    @Override
    public void execute() {

        // this will just print stuff over and over when the command is active.
        System.out.println("CoralIntakeIn activated");


    }

    @Override
    public boolean isFinished() {
      //make sure "return false;" is here or else it will never stop running. (i think)
      return false;
    }
  
    @Override
    public void end(boolean interrupted) {
        intakeIn.coralStop();
    }
}