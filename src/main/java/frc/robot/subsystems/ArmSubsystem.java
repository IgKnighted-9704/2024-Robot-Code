package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmSubsystem extends PIDSubsystem {

  private static final int ARM_LEFT_ID = 9;

  public static final double kARM_FLOOR_POS = 0.0; // intaking
  public static final double kARM_FENDER_POS = 0.190573; // close shot
  public static final double kARM_HIGH_POS = 0.190573; // high shot
  public static final double kARM_START_POS = 0.3; // start config
  public static final double kARM_AMP_POS = 2.933511; // amp scoring

  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;

  public ArmSubsystem() {
    super(new PIDController(2.7, 0.0, 0.0));  // PID values from SmartDashboard
    armMotor = new CANSparkMax(ARM_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);  // Arm motor ID 9
    armEncoder = armMotor.getEncoder();

    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor.setOpenLoopRampRate(0.25);

    moveToShoot();

    SmartDashboard.putNumber("Arm kP", getController().getP());
    SmartDashboard.putNumber("Arm kI", getController().getI());
    SmartDashboard.putNumber("Arm kD", getController().getD());
    SmartDashboard.putNumber("Arm kI Zone", getController().getIZone());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    final double maxPower = 0.5;

    // Limit power to prevent excessive torque
    if (output > maxPower) {
        output = maxPower;
    } else if (output < -maxPower) {
        output = -maxPower;
    }   
    armMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    return armEncoder.getPosition();
  }

  public void moveToAmp() {
    getController().setSetpoint(kARM_AMP_POS);
  }

  public void moveToShoot() {
    getController().setSetpoint(kARM_HIGH_POS);
  }

  public void moveToFloor() {
    getController().setSetpoint(kARM_FLOOR_POS);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void resetArmEncoder() {
    armEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), getSetpoint());
    }
    SmartDashboard.putNumber("Arm", getMeasurement());
    SmartDashboard.putNumber("Arm Target", getController().getSetpoint());
    // comment these out when not needed
    getController().setP(SmartDashboard.getNumber("Arm kP", 2.7));
    getController().setI(SmartDashboard.getNumber("Arm kI", 0));
    getController().setD(SmartDashboard.getNumber("Arm kD", 0));
    getController().setIZone(SmartDashboard.getNumber("Arm kI Zone", 0));
  }
}
