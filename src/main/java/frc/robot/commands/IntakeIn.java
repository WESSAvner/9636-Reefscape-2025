package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeIn extends Command {
    public final Intake m_intakeIn;

    public IntakeIn(Intake intakeCommand) {

        m_intakeIn = intakeCommand;

    }

    @Override
    public void initialize() {

        m_intakeIn.m_robotIntake(1);
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
      m_intakeIn.m_robotIntakeStop();
    }
}