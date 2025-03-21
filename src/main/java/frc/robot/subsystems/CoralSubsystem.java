package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;


public class CoralSubsystem extends SubsystemBase {
    private SparkMax coralMotorLeft = new SparkMax(5, MotorType.kBrushless);
    private SparkMax coralMotorRight = new SparkMax(6, MotorType.kBrushless);
    private DigitalInput coralSensor = new DigitalInput(2);

    private boolean coralTooketh = false;

    public void coralIntakeIn() {

        coralMotorLeft.set(0.5);
        coralMotorRight.set(-0.5);

        if (coralSensor.get()) {

            coralTooketh = true;

            coralMotorLeft.set(0);
            coralMotorRight.set(0);
            return;

        }

    }

    public void coralOutput() {

        coralMotorLeft.set(0.5);
        coralMotorRight.set(-0.5);

        coralTooketh = false;

    }

}