package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The ShooterSubsystem controls the shooting mechanism of the robot,
 * including the shooter wheels, intake motor, and associated sensors.
 */
public class ShooterSubsystem extends SubsystemBase {
    // Constants for CAN IDs and sensor IDs
    private static final int SHOOTER_A_ID = 5;    // Shooter motor A CAN ID
    private static final int SHOOTER_B_ID = 7;    // Shooter motor B CAN ID
    private static final int INTAKE_ID = 3;       // Intake motor CAN ID
    private static final int SENSOR_ID = 1;       // Entry sensor digital input ID
    private static final int SENSOR2_ID = 0;      // Shooter sensor digital input ID

    // Shooter motors
    private final CANSparkMax shooterA;
    private final CANSparkMax shooterB;
    // Intake motor
    private final CANSparkMax intakeMotor;
    // Sensors for detecting notes
    private final DigitalInput sensor;
    private final DigitalInput sensor2;
    // Reference to the ArmSubsystem
    private final ArmSubsystem armSubsystem;
    // PID controller for shooter wheel speed
    private final PIDController shooterPID;
    // Feedforward controller for shooter wheel speed
    private final SimpleMotorFeedforward shooterFeedforward;
    // Flag to indicate whether the robot is feeding notes
    private boolean feeding = false;

    /**
     * Constructs a new ShooterSubsystem.
     *
     * @param armSubsystem The arm subsystem used to control the robot's arm.
     */
    public ShooterSubsystem(ArmSubsystem armSubsystem) {
        // Initialize shooter motors
        shooterA = new CANSparkMax(SHOOTER_A_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterB = new CANSparkMax(SHOOTER_B_ID, CANSparkLowLevel.MotorType.kBrushless);
        // Initialize intake motor
        intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);
        // Initialize feedforward and PID controllers for shooter speed control
        shooterFeedforward = new SimpleMotorFeedforward(0.011, 0.0001762, 0);
        shooterPID = new PIDController(0.0001, 0, 0);
        // Initialize sensors
        sensor = new DigitalInput(SENSOR_ID);
        sensor2 = new DigitalInput(SENSOR2_ID);
        // Store reference to arm subsystem
        this.armSubsystem = armSubsystem;

        // Optionally set encoder conversion factors, burn settings to flash
        // shooterA.getEncoder().setVelocityConversionFactor(1);
        // shooterB.getEncoder().setVelocityConversionFactor(1);
        // shooterA.burnFlash();
        // shooterB.burnFlash();

        // Put default shooter target RPM to SmartDashboard
        SmartDashboard.putNumber("Shooter Target", 3400);
    }

    /**
     * Gets the current average shooter speed in RPM.
     *
     * @return The average speed of the shooter motors in RPM.
     */
    public double getShooterSpeed() {
        // Get the negative average of the velocities from both shooter motors
        return -(shooterA.getEncoder().getVelocity() + shooterB.getEncoder().getVelocity()) / 2;
        // Alternatively, return shooterA.getEncoder().getVelocity();
    }

    /**
     * Controls the shooter motors using PID and feedforward to reach the desired RPM.
     *
     * @param RPM The target RPM for the shooter wheels.
     */
    public void shootPID(double RPM) {
        // Calculate PID output
        double pidOutput = shooterPID.calculate(getShooterSpeed(), RPM);
        // Calculate feedforward output
        double feedforwardOutput = shooterFeedforward.calculate(RPM);
        // Set the shooter motors with the combined output (negative to adjust direction)
        shooterA.set(-(pidOutput + feedforwardOutput));
        shooterB.set(-(pidOutput + feedforwardOutput));
    }

    /**
     * Runs the shooter motors at the specified power level.
     *
     * @param power The power level to run the shooter motors at (-1.0 to 1.0).
     */
    public void shoot(double power) {
        // Set shooter motors to the specified power (negative to adjust direction)
        shooterA.set(-power);
        shooterB.set(-power);
    }

    /**
     * Stops the shooter motors and the intake motor.
     */
    public void stopShooter() {
        shooterA.stopMotor();
        shooterB.stopMotor();
        stopIntake();
    }

    /**
     * Controls the intake mechanism based on sensor inputs.
     * Manages the arm position and intake motor power.
     */
    public void intake() {
        boolean entrySensor = getEntrySensor();
        boolean shooterSensor = getShooterSensor();

        if (entrySensor && !shooterSensor) {
            // Note is at the entry sensor but not at the shooter sensor
            // Move the arm to the fender position and run the intake motor
            armSubsystem.moveToFender();
            intakeMotor.set(-1.0);
        } else if (!entrySensor && shooterSensor) {
            // Note has moved past the entry sensor and is at the shooter sensor
            // Stop the intake motor
            stopIntake();
        } else if (entrySensor && shooterSensor) {
            // Note is detected at both sensors
            // Stop the intake motor
            stopIntake();
        } else if (!entrySensor && !shooterSensor) {
            // No note detected at either sensor
            // Move the arm to the floor position and run the intake motor
            armSubsystem.moveToFloor();
            intakeMotor.set(-1.0);
        }
    }

    /**
     * Stops the intake motor and shooter motors.
     * If the arm is below a certain position, moves the arm to the shooting position.
     */
    public void stopIntake() {
        // Stop the intake motor
        intakeMotor.stopMotor();
        // Stop the shooter motors
        shoot(0.0);
        // If the arm is below the fender position, move it to the shooting position
        if (armSubsystem.getMeasurement() < ArmSubsystem.kARM_FENDER_POS) {
            armSubsystem.moveToShoot();
        }
    }

    /**
     * Reverses the intake and shooter motors to outtake notes.
     */
    public void outtake() {
        // Reverse the intake motor
        intakeMotor.set(0.5);
        // Reverse the shooter motors
        shoot(-0.5);
    }

    /**
     * Outtakes the note if the shooter sensor detects a note.
     */
    public void sensorOuttake() {
        if (getShooterSensor()) {
            // If a note is at the shooter sensor, outtake
            intakeMotor.set(0.5);
            shoot(-0.5);
        }
    }

    /**
     * Prepares the shooter to shoot notes into the speaker.
     * Runs the intake motor and sets the shooter to target RPM.
     */
    public void shootInSpeaker() {
        // Run the intake motor
        intakeMotor.set(-1.0);
        if (feeding) {
            // If feeding, use higher target RPM
            shootPID(3400);
        } else {
            // If not feeding, use lower target RPM
            shootPID(3600);
        }
    }

    /**
     * Spins up the shooter to prepare for shooting.
     * Moves the arm to the shooting position.
     */
    public void spinUpShooter() {
        feeding = false;
        armSubsystem.moveToShoot();
        shootPID(3600);
    }

    /**
     * Spins up the shooter to the feed position.
     * Moves the arm to the feeding position.
     */
    public void spinUpFeed() {
        feeding = true;
        armSubsystem.moveToFeed();
        // Use shooter target RPM from SmartDashboard
        shootPID(SmartDashboard.getNumber("Shooter Target", 3400));
        // Alternatively, run the shooter at full power
        // shoot(1);
    }

    /**
     * Checks if the entry sensor is triggered.
     *
     * @return True if the entry sensor is triggered (note is present), false otherwise.
     */
    public boolean getEntrySensor() {
        return !sensor.get();
    }

    /**
     * Checks if the shooter sensor is triggered.
     *
     * @return True if the shooter sensor is triggered (note is present), false otherwise.
     */
    public boolean getShooterSensor() {
        return !sensor2.get();
    }

    /**
     * Periodically called method to update sensor readings and output to SmartDashboard.
     */
    @Override
    public void periodic() {
        // Output sensor states to SmartDashboard
        SmartDashboard.putBoolean("Entry Sensor", getEntrySensor());
        SmartDashboard.putBoolean("Shooter Sensor", getShooterSensor());
        // Output current shooter RPM to SmartDashboard
        SmartDashboard.putNumber("Shooter RPM", getShooterSpeed());
    }
}
