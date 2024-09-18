package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    SmartDashboard.putNumber("Arm kP", 2.7);
    SmartDashboard.putNumber("Arm kI", 0);
    SmartDashboard.putNumber("Arm kD", 0);
    SmartDashboard.putNumber("Arm kI Zone", 0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    double limitedOutput = Math.max(-0.5, Math.min(0.5, output));  // Limit power
    armMotor.set(limitedOutput);
  }

  @Override
  public double getMeasurement() {
    return armEncoder.getPosition();
  }

  public void moveArm(double power) {
    double limitedPower = Math.max(-0.5, Math.min(0.5, power));
    armMotor.set(limitedPower);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void resetArmEncoder() {
    armEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    getController().setP(SmartDashboard.getNumber("Arm kP", 2.7));
    getController().setI(SmartDashboard.getNumber("Arm kI", 0));
    getController().setD(SmartDashboard.getNumber("Arm kD", 0));
    getController().setIntegratorRange(0, SmartDashboard.getNumber("Arm kI Zone", 0));
  }
}
