package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;


public class CoralSubsystem extends SubsystemBase {

    // this is how you instanciate a SparkMax. Device IDs are set in the rev hardware client.
    private SparkMax coralMotorLeft = new SparkMax(5, MotorType.kBrushless);
    private SparkMax coralMotorRight = new SparkMax(6, MotorType.kBrushless);

    // this stuff is for the sensor we were supposed to use, but we didnt, so its unused.
    private DigitalInput coralSensor = new DigitalInput(2);
    private boolean coralTooketh = false;

    public void coralIntakeIn() {

        // the "motor.set(speed:);" command sets the motors to a specific speed from -1 to 1. these are going in opposite direction because of the physical orientation of them on the robot.
        coralMotorLeft.set(-0.3);
        coralMotorRight.set(0.3);

        // if (coralSensor.get()) {

        //     coralTooketh = true;

        //     coralMotorLeft.set(0);
        //     coralMotorRight.set(0);
        //     return;

        // }

    }

    public void coralOutput() {

        coralMotorLeft.set(0.3);
        coralMotorRight.set(-0.3);

        // coralTooketh = false;

    }

    public void coralStop() {

            coralMotorLeft.set(0);
            coralMotorRight.set(0);

    }

}