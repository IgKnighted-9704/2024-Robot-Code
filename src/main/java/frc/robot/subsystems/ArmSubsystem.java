package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

/**
 * The ArmSubsystem controls the movement of the robot's arm using a PID controller.
 * It allows the arm to move to predefined positions for various actions like intaking,
 * shooting, and feeding.
 */
public class ArmSubsystem extends PIDSubsystem {

    // CAN ID for the arm motor controller
    private static final int ARM_LEFT_ID = 9;

    // Predefined arm positions (encoder counts or rotations)
    public static final double kARM_FLOOR_POS = 0.0;              // Position for intaking from the floor
    public static final double kARM_FENDER_POS = 0.05;            // Position for close-range shooting (fender)
    public static final double kARM_HIGH_POS = 0.190573;          // Position for high goal shooting
    public static final double kARM_START_POS = 0.3;              // Starting configuration position
    public static final double kARM_AMP_POS = 3.01;               // Position for amplifier scoring
    public static final double kARM_FEED_POS = 0.462996697425842; // Position for feeding

    // Motor controller for the arm
    private final CANSparkMax armMotor;

    // Encoder attached to the arm motor for position feedback
    private final RelativeEncoder armEncoder;

    /**
     * Constructs a new ArmSubsystem.
     * Initializes the motor controller, encoder, and PID controller.
     */
    public ArmSubsystem() {
        // Initialize the PIDSubsystem with a PIDController (kP, kI, kD)
        super(new PIDController(2.7, 0.0, 0.0));

        // Set the integral zone (I-Zone) for the PID controller to zero
        getController().setIZone(0);

        // Create a new motor controller for the arm motor with the specified CAN ID
        armMotor = new CANSparkMax(ARM_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);

        // Get the encoder from the motor controller
        armEncoder = armMotor.getEncoder();

        // Set the motor to brake mode to hold position when no power is applied
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the ramp rate for open-loop control to limit acceleration
        armMotor.setOpenLoopRampRate(0.25);

        // Move the arm to the shooting position upon initialization
        moveToShoot();

        // Optionally, display the PID coefficients on the SmartDashboard for tuning
        // SmartDashboard.putNumber("Arm kP", getController().getP());
        // SmartDashboard.putNumber("Arm kI", getController().getI());
        // SmartDashboard.putNumber("Arm kD", getController().getD());
        // SmartDashboard.putNumber("Arm kI Zone", getController().getIZone());
    }

    /**
     * Uses the output from the PID controller to control the arm motor.
     * This method is automatically called by the PIDSubsystem.
     *
     * @param output   The output value from the PID controller.
     * @param setpoint The target setpoint (desired position).
     */
    @Override
    public void useOutput(double output, double setpoint) {
        final double maxPower = 0.5;

        // Limit the output power to prevent excessive torque and potential damage
        if (output > maxPower) {
            output = maxPower;
        } else if (output < -maxPower) {
            output = -maxPower;
        }

        // Command the motor to move with the calculated output power
        armMotor.set(output);
    }

    /**
     * Returns the current position of the arm for feedback in the PID controller.
     *
     * @return The current position from the arm encoder.
     */
    @Override
    public double getMeasurement() {
        return armEncoder.getPosition();
    }

    /**
     * Commands the arm to move to the amplifier scoring position.
     */
    public void moveToAmp() {
        getController().setSetpoint(kARM_AMP_POS);
    }

    /**
     * Commands the arm to move to the high goal shooting position.
     */
    public void moveToShoot() {
        getController().setSetpoint(kARM_HIGH_POS);
    }

    /**
     * Commands the arm to move to the floor position for intaking.
     */
    public void moveToFloor() {
        getController().setSetpoint(kARM_FLOOR_POS);
    }

    /**
     * Commands the arm to move to the fender position for close-range shooting.
     */
    public void moveToFender() {
        getController().setSetpoint(kARM_FENDER_POS);
    }

    /**
     * Commands the arm to move to the feeding position.
     */
    public void moveToFeed() {
        getController().setSetpoint(kARM_FEED_POS);
    }

    /**
     * Stops the arm motor immediately.
     */
    public void stopArm() {
        armMotor.stopMotor();
    }

    /**
     * Resets the arm encoder's position to zero.
     * Useful for calibration at startup.
     */
    public void resetArmEncoder() {
        armEncoder.setPosition(0.0);
    }

    /**
     * Called periodically by the scheduler.
     * Updates the PID controller and outputs telemetry data to the SmartDashboard.
     */
    @Override
    public void periodic() {
        // Calculate the PID output and apply it to the motor
        useOutput(m_controller.calculate(getMeasurement()), getSetpoint());

        // Display the current arm position and target position on the SmartDashboard
        SmartDashboard.putNumber("Arm", getMeasurement());
        SmartDashboard.putNumber("Arm Target", getController().getSetpoint());

        // Optionally, update the PID coefficients from the SmartDashboard for live tuning
        // Uncomment these lines to enable live PID tuning
        // getController().setP(SmartDashboard.getNumber("Arm kP", getController().getP()));
        // getController().setI(SmartDashboard.getNumber("Arm kI", getController().getI()));
        // getController().setD(SmartDashboard.getNumber("Arm kD", getController().getD()));
        // getController().setIZone(SmartDashboard.getNumber("Arm kI Zone", getController().getIZone()));
    }

}
