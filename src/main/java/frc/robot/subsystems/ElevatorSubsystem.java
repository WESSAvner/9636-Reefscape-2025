package frc.robot.subsystems;

import javax.security.auth.callback.ConfirmationCallback;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax primaryMotor = new SparkMax(3, MotorType.kBrushless);
    private SparkMax followerMotor = new SparkMax(4, MotorType.kBrushless);
    private RelativeEncoder encoder = primaryMotor.getEncoder();
    private DigitalInput bottomLimit = new DigitalInput(5);

    private PIDController pidController = new PIDController(
        0.01,
        0,
        0.01
    );

    public ElevatorSubsystem() {
        if (!motorsConfigured) {
            configureMotors();
            motorsConfigured = true;
        }
    }
    

    // private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 1));

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;

    boolean motorsConfigured = false;

    


    public enum ElevatorPosition {
        DOWN(0),
        POSITION_1(10),
        POSITION_2(20),
        POSITION_3(30),
        POSITION_4(40);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }


    
    private void configureMotors() {

        pidController.setTolerance(0.5);

        // Primary motor configuration
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, true);
        followerMotor.configure(followerConfig, null, null); 
        
        // Follower motor configuration
        // followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }
   
    @Override
    public void periodic() {

        // Configures motors at start.
  

        if (!isHomed && bottomLimit.get()) {
            handleBottomLimit();
        }

        currentPos = encoder.getPosition() / Constants.countsPerInch;
        
        // Calculate the next state and update current state
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        if (currentPos < 0.05) {
            handleBottomLimit();
        }

        if (getHeightInches() > 40) {
            stopMotors();
        }

        // Only run control if homed
        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput + ff,
                -0.5,
                0.5
            );
            
            primaryMotor.set(outputPower);
        }

        // Update SmartDashboard
        updateTelemetry();
    }

    private void handleBottomLimit() {
        stopMotors();
        // encoder.setPosition(0 * Constants.countsPerInch);
        isHomed = true;
        setpoint = 0;  
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < 1;
    }
    

    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        return 0 * Math.signum(state.velocity) +
               0.01 +
               .1 * state.velocity;
    }

    // Sets position in inches (AMERICA RAAAAAAAHHHH)
    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            0,
            40
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    public double getHeightInches() {
        return encoder.getPosition() / Constants.countsPerInch;
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
    }



    public void homeElevator() {
        if (!isHomed) {
            primaryMotor.set(-0.1); // Slow downward movement
        }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public ElevatorPosition getCurrentTarget() {
        return currentTarget;
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
        if (!isHomed && power < 0) {
            power = 0;
        }
        
        if (getHeightInches() >= 67 && power > 0) {
            power = 0;
        }
        
        if (bottomLimit.get() && power < 0) {
            power = 0;
        }
        
        primaryMotor.set(MathUtil.clamp(power, -0.5, 0.5));
    }
}