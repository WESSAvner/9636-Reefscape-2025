package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.AutoCloseable;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;



public class AlgaeIntake extends SubsystemBase {
    SparkMax m_intakeMotorLead = new SparkMax(5, MotorType.kBrushless);
    SparkMax m_intakeMotorFollow = new SparkMax(6, MotorType.kBrushless);



    public void m_robotIntake(double speed) {
        m_intakeMotorLead.set(speed);
        m_intakeMotorFollow.set(-speed);
    }
        
    public void m_robotIntakeOut(double speed) {
        m_intakeMotorLead.set(-speed);
        m_intakeMotorFollow.set(speed);
    }      

    public void m_robotIntakeStop() {
        m_intakeMotorLead.set(0);
        m_intakeMotorFollow.set(0);
    }

}