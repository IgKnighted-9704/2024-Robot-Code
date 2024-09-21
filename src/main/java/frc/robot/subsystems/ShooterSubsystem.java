package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static final int SHOOTER_A_ID = 5;
  private static final int SHOOTER_B_ID = 7;
  private static final int INTAKE_ID = 3;
  private static final int SENSOR_ID = 1;
  private static final int SENSOR2_ID = 0;

  private final CANSparkMax shooterA;
  private final CANSparkMax shooterB;
  private final CANSparkMax intakeMotor;
  private final DigitalInput sensor;
  private final DigitalInput sensor2;
  private final ArmSubsystem armSubsystem;

  public ShooterSubsystem(ArmSubsystem armSubsystem) {
    shooterA = new CANSparkMax(SHOOTER_A_ID, CANSparkLowLevel.MotorType.kBrushless);  // Shooter A motor ID 5
    shooterB = new CANSparkMax(SHOOTER_B_ID, CANSparkLowLevel.MotorType.kBrushless);  // Shooter B motor ID 7
    intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);  // Intake motor ID 3
    sensor = new DigitalInput(SENSOR_ID);  // Sensor ID 1
    sensor2 = new DigitalInput(SENSOR2_ID);  // Sensor2 ID 0
    this.armSubsystem = armSubsystem;
  }

  public void shoot(double power) {
    shooterA.set(-power);  // Negative power for shooter
    shooterB.set(-power);  // Synchronizing both shooter motors
  }

  public void stopShooter() {
    shooterA.stopMotor();
    shooterB.stopMotor();
    stopIntake();
  }

  public void intake() {
    if (getEntrySensor()) {
      if (getShooterSensor()) {
        stopIntake();
        return;
      }
      armSubsystem.moveToShoot();
    } else {
      armSubsystem.moveToFloor();
    }
    if (getShooterSensor()) {
        stopIntake();
        return;
    }
    intakeMotor.set(-1.0);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    shoot(0.0);
    if (armSubsystem.getMeasurement() < ArmSubsystem.kARM_FENDER_POS)
      armSubsystem.moveToShoot();
  }

  public void outtake() {
    intakeMotor.set(0.5);
    shoot(-0.5);
  }

  public void shootInSpeaker() {
    intakeMotor.set(-1.0);
    shoot(0.65);
  }

  public void spinUpShooter() {
    armSubsystem.moveToShoot();
    shoot(0.65);
  }

  public boolean getEntrySensor() {
    return !sensor.get();
  }

  public boolean getShooterSensor() {
    return !sensor2.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Sensor", getEntrySensor());
    SmartDashboard.putBoolean("Sensor Shooter", getShooterSensor());
  }
}
