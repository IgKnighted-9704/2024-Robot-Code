package frc.robot.subsystems.manipulators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;

public class Manipulator {
    private static final int ARM_LEFT_ID = 5;
    private static final int ARM_RIGHT_ID = 6;
    private static final int SHOOTER_A_ID = 7;
    private static final int SHOOTER_B_ID = 8;
    private static final int INTAKE_ID = 9;
    private static final int SENSOR_ID = 0;

    public static final double kARM_FLOOR_POS = 0.584; // intaking
    public static final double kARM_FENDER_POS = 0.53; // close shot
    public static final double kARM_HIGH_POS = 0.53; // high shot
    public static final double kARM_START_POS = 0.376; // start config
    public static final double kARM_AMP_POS = 0.325; // amp scoring

    private CANSparkMax armLeft;
    private CANSparkMax armRight;
    private CANSparkMax shooterA;
    private CANSparkMax shooterB;
    private CANSparkMax intakeMotor;

    private DigitalInput sensor;

    private AbsoluteEncoder armEncoder;

    private Manipulator() {
        armLeft = new CANSparkMax(ARM_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
        armRight = new CANSparkMax(ARM_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterA = new CANSparkMax(SHOOTER_A_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterB = new CANSparkMax(SHOOTER_B_ID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);

        sensor = new DigitalInput(SENSOR_ID);

        armEncoder = armLeft.getAbsoluteEncoder();

        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armLeft.setOpenLoopRampRate(0.25);
        armRight.setOpenLoopRampRate(0.25);
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

        armLeft.set(-power);
        armRight.set(-power); // Negative power because motors are pointing in opposite directions
    }

    public void armToPosition(double position) {
        double kP = -15.0;
        double error = position - armEncoder.getPosition();
        double power = kP * error;
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

    public boolean getNoteSensor() {
        return sensor.get();
    }
}

