package frc.robot.subsystems.manipulators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase {
    private final int SHOOTER_A_ID = 5;
    private final int SHOOTER_B_ID = 7;
    private final int INTAKE_ID = 3;
    private final int SENSOR_ID = 1;
    private final int SENSOR2_ID = 0;

    private final CANSparkMax shooterA = new CANSparkMax(SHOOTER_A_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterB = new CANSparkMax(SHOOTER_B_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);

    private final DigitalInput sensor = new DigitalInput(SENSOR_ID);
    private final DigitalInput sensor2 = new DigitalInput(SENSOR2_ID);

    public ShooterSubsytem() {
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public Command runIntakeCommand() {
        return this.startEnd(() -> intakeMotor.set(1.0), () -> intakeMotor.set(0.0));
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
