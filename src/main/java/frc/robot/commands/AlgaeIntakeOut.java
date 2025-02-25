package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;


public class AlgaeIntakeOut extends Command{
    public final AlgaeIntake m_intakeOut;

    public AlgaeIntakeOut(AlgaeIntake intakeCommand) {

        m_intakeOut = intakeCommand;

    }

   @Override
    public void initialize() {

        m_intakeOut.m_robotIntakeOut(1);
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
      m_intakeOut.m_robotIntakeStop();
    }
}