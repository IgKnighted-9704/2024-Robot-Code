package frc.robot.subsystems.manipulators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
    private static final int ARM_LEFT_ID = 9;
    private static final int SHOOTER_A_ID = 5;
    private static final int SHOOTER_B_ID = 7;
    private static final int INTAKE_ID = 3;
    private static final int SENSOR_ID = 1;
    private static final int SENSOR2_ID = 0;


    public static final double kARM_FLOOR_POS = 0.0; // intaking
    public static final double kARM_FENDER_POS = 0.190573; // close shot
    public static final double kARM_HIGH_POS = 0.190573; // high shot
    public static final double kARM_START_POS = 0.3; // start config
    public static final double kARM_AMP_POS = 2.933511; // amp scoring

    private CANSparkMax armLeft;
    private CANSparkMax shooterA;
    private CANSparkMax shooterB;
    private CANSparkMax intakeMotor;

    private DigitalInput sensor;
    private DigitalInput sensor2;
    private PIDController pid;


    private RelativeEncoder armEncoder;

    private Manipulator() {
        armLeft = new CANSparkMax(ARM_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterA = new CANSparkMax(SHOOTER_A_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterB = new CANSparkMax(SHOOTER_B_ID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);
        SmartDashboard.putNumber("Arm kP", 2.7);
        SmartDashboard.putNumber("Arm kI", 0);
        SmartDashboard.putNumber("Arm kD", 0);
        SmartDashboard.putNumber("Arm kI Zone", 0);
        pid = new PIDController(0.0, 0.0, 0.0);

        sensor = new DigitalInput(SENSOR_ID);
        sensor2 = new DigitalInput(SENSOR2_ID);

        armEncoder = armLeft.getEncoder();

        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armLeft.setOpenLoopRampRate(0.25);
    }

    public static Manipulator getInstance() {
        return ManipulatorHolder.INSTANCE;
    }

    private static class ManipulatorHolder {
        private static final Manipulator INSTANCE = new Manipulator();
    }

    public void moveArm(double power) {
        final double maxPower = 0.5;

        // Limit power to prevent excessive torque
        if (power > maxPower) {
            power = maxPower;
        } else if (power < -maxPower) {
            power = -maxPower;
        }

        armLeft.set(power);
    }



    public void armToPosition(double position) {
        pid.setPID(SmartDashboard.getNumber("Arm kP", 0), SmartDashboard.getNumber("Arm kI", 0), SmartDashboard.getNumber("Arm kD", 0));
        pid.setIZone(SmartDashboard.getNumber("Arm kI Zone", 0));
        double power = pid.calculate(armEncoder.getPosition(), position);
        // double kP = SmartDashboard.getNumber("Arm PID", 2.7);
        // double error = position - armEncoder.getPosition();
        // double power = kP * error;
        moveArm(power);
    }

    public void intake(double power) {
        intakeMotor.set(power);
    }

    public void shoot(double power) {
        shooterA.set(-power);
        shooterB.set(-power); // TODO: Check polarities
    }

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public void resetArmEncoder() {
        armEncoder.setPosition(0.0);
    }

    public boolean getNoteEntrySensor() {
        return !sensor.get();
    }

    public boolean getNoteShooterSensor() {
        return !sensor2.get();
    }
}

