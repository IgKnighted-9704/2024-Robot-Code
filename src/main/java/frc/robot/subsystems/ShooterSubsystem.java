package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private final PIDController shooterPID;
  private final SimpleMotorFeedforward shooterfeedforward;
  private boolean feeding = false;

  public ShooterSubsystem(ArmSubsystem armSubsystem) {
    shooterA = new CANSparkMax(SHOOTER_A_ID, CANSparkLowLevel.MotorType.kBrushless);  // Shooter A motor ID 5
    shooterB = new CANSparkMax(SHOOTER_B_ID, CANSparkLowLevel.MotorType.kBrushless);  // Shooter B motor ID 7
    intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);  // Intake motor ID 3
    shooterfeedforward = new SimpleMotorFeedforward(0.011, 0.0001762, 0);
    shooterPID = new PIDController(0.0001, 0, 0);
    sensor = new DigitalInput(SENSOR_ID);  // Sensor ID 1
    sensor2 = new DigitalInput(SENSOR2_ID);  // Sensor2 ID 0
    this.armSubsystem = armSubsystem;
    // shooterA.getEncoder().setVelocityConversionFactor(1);
    // shooterB.getEncoder().setVelocityConversionFactor(1);
    // shooterA.burnFlash();
    // shooterB.burnFlash();
    SmartDashboard.putNumber("Shooter Target", 4000);
  }

  public double getShooterSpeed() {
      return -(shooterA.getEncoder().getVelocity() + shooterB.getEncoder().getVelocity()) / 2;
    // return shooterA.getEncoder().getVelocity();
    }

  public void shootPID(double RPM) {
    shooterA.set(-(shooterPID.calculate(getShooterSpeed(), RPM) + shooterfeedforward.calculate(RPM)));
    shooterB.set(-(shooterPID.calculate(getShooterSpeed(), RPM) + shooterfeedforward.calculate(RPM)));
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
    boolean entrySensor = getEntrySensor();
    boolean shooterSensor = getShooterSensor();

    if (entrySensor && !shooterSensor) {
        armSubsystem.moveToFender();
        intakeMotor.set(-1.0);
    } else if (!entrySensor && shooterSensor) {
        // outtake();
        stopIntake();
    } else if (entrySensor && shooterSensor) {
        stopIntake();
    } else if (!entrySensor && !shooterSensor) {
        armSubsystem.moveToFloor();
        intakeMotor.set(-1.0);
    }
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

  public void sensorOuttake() {
    if (getShooterSensor()) {
    intakeMotor.set(0.5);
    shoot(-0.5);
    }
  }

  public void shootInSpeaker() {
    intakeMotor.set(-1.0);
    if (feeding)
      shootPID(4000);
    else
      shootPID(3600);
  }

  public void spinUpShooter() {
    feeding = false;
    armSubsystem.moveToShoot();
    shootPID(3600);
  }

  public void spinUpFeed() {
    feeding = true;
    armSubsystem.moveToFeed();
    shootPID(SmartDashboard.getNumber("Shooter Target", 4000));
    // shoot(1);
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
    SmartDashboard.putNumber("Shooter RPM", getShooterSpeed());
  }
}
