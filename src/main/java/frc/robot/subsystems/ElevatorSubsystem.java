package frc.robot.subsystems;

// import javax.security.auth.callback.ConfirmationCallback;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax primaryMotor = new SparkMax(3, MotorType.kBrushless);
    private SparkMax followerMotor = new SparkMax(4, MotorType.kBrushless);
    private RelativeEncoder encoder = primaryMotor.getEncoder();
    // private DigitalInput bottomLimit = new DigitalInput(5);

    private PIDController pidController = new PIDController(
        Constants.elevatorkP,
        Constants.elevatorkI,
        Constants.elevatorkD
    );

    //Configures motors
    boolean motorsConfigured = false;

    public ElevatorSubsystem() {
        if (!motorsConfigured) {
            configureMotors();
            motorsConfigured = true;
        }
    }
    

    // private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(20, 15));

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    public static double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;



    

    // No clue what this is, but its there.
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


    /*  
    Configures the primary and follower motors and sets a tolerance to
    the PID for how close it can be/should be to the point. 
    */
    private void configureMotors() {

        // Setting tolerance, why is this in configure motors???
        pidController.setTolerance(0.5);

        // Primary motor configuration
        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);

        // Follower motor configuration
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, true);
        followerMotor.configure(followerConfig, null, null); 
        
    }
   
    // periodic() happens every 20ms, so anything that needs to be repeated constantly goes here.
    @Override
    public void periodic() {

        // Gives a variable for the current positon of the elevator carrage
        currentPos = encoder.getPosition() / Constants.countsPerInch;
  
        // Handles the bottom of the elevator when position is below 0.5 and isHomed = false
        if (!isHomed && currentPos < 0.5) {
            handleBottomLimit();
        }

        
        // Calculate the next state and update current state
        currentState = profile.calculate(Constants.elevatorPIDLoopTime, currentState, goalState); // 20ms control loop

        // handles bottom limit if position is close to zero
        // if (currentPos < 0.05) {
        //     handleBottomLimit();
        // }

        // Stops the motors if the height gets too far. may we all pray this works /\
        if (getHeightInches() > 10) {
            stopMotors();
        }

        // Only runs the motors if homed and it knows where position zero is.
        if (isHomed) {
            // Read up on PID stuff, this is weird.
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            // clamps the output power to be between -0.5 and 0.5 so no one dies
            double outputPower = pidOutput + ff;
            
            // takes the resulting output power and powers the motor
            primaryMotor.set(outputPower);

        }

        // Update SmartDashboard
        updateTelemetry();
    }

    /* 
    Stops the motor movement, sets isHomed to true, setpoint to 0,
    defines the current and goal state, and resets the PID
    */
    private void handleBottomLimit() {
        stopMotors();
        isHomed = true;
        setpoint = 0;  
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        pidController.reset();
    }

    // Take a WILD guess as to what this does.
    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    // Checks if the elevator is within a small tolerance of the target height
    public boolean isAtHeight(double targetHeightInches) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < 1;
    }
    
    // 0.01 is kS, 0.1 is kG, and state.velocity is kV
    // kS (static friction), kG (gravity), kV (velocity),
    private double calculateFeedForward(TrapezoidProfile.State state) {
        return 0 * Math.signum(state.velocity) +
               Constants.elevatorkS +
               Constants.elevatorkG * state.velocity;
    }

    // This takes in a value (inches) and changes the setpoint that is to be the goalState
    public void setPositionInches(double inches) {
        System.out.println("Setting Elevator Setpoint: " + inches);
        // Gives warning if elevator is not homed to console
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        // Limits the possible input of inches
        setpoint = MathUtil.clamp(
            inches,
            0,
            Constants.elevatorMaxHeight // Max Height
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
        System.out.println("Updated goal state: " + goalState);
    }

    // gets the current height of the carrage.
    public double getHeightInches() {
        return encoder.getPosition() / Constants.countsPerInch;
    }

    // gives smartdashboard some numbers for "easy" debugging
    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);

    }


    // From here and downward, these methods are never used. perhaps a use for them will be found later on.

    public void homeElevator() {
        if (!isHomed) {
            primaryMotor.set(-0.1); // Slow downward movement
        }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isElevatorHomed() {
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
        
        // if (bottomLimit.get() && power < 0) {
        //     power = 0;
        // }
        
        primaryMotor.set(MathUtil.clamp(power, -0.5, 0.5));
    }
}