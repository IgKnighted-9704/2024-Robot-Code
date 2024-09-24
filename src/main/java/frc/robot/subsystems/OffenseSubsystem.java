package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OffenseSubsystem extends SubsystemBase {
    private final CANSparkMax intake;
    private final CANSparkMax shootMotor1;
    private final CANSparkMax shootMotor2;
    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final PIDController pidController = new PIDController(Constants.SubsystemContants.kP_Arm,Constants.SubsystemContants.kI_Arm,Constants.SubsystemContants.kD_Arm);
    private final DigitalInput intakeSensor;
    private final DigitalInput shootSensor;


    public OffenseSubsystem(int IntakeId, int SM1, int SM2,int ArmId, int intakeSensor, int shootSensor){
        intake = new CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushless);
        shootMotor1 = new CANSparkMax(SM1, CANSparkLowLevel.MotorType.kBrushless);
        shootMotor2 = new CANSparkMax(SM2, CANSparkLowLevel.MotorType.kBrushless);
        armMotor = new CANSparkMax(ArmId,CANSparkLowLevel.MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        this.intakeSensor = new DigitalInput(intakeSensor);
        this.shootSensor = new DigitalInput(shootSensor);

        raiseArm2Position(Constants.ArmSetupConstants.ARM_DefaultPos);
    }

    public void shoot(double speed){
        if(getShooterSensor() && getEntrySensor()){
            shootMotor1.set(speed);
            shootMotor2.set(-speed);
        }
    }

    public void intake(double speed){
        if(getEntrySensor()){
            if(getShooterSensor()){
                intake.stopMotor();
            }      
            intake.set(-speed); 
        }
    }

    public void outtake(double speed){
        shootMotor1.set(-speed);
        shootMotor2.set(speed);
        intake.set(speed);
    }

    public void raiseArm2Position(double targetPos){
        pidController.setSetpoint(targetPos);
        double power = pidController.calculate(armEncoder.getPosition());
        armMotor.set(power);
    }

    public void raiseArm(double speed){
        armMotor.set(speed);
    }

    public boolean getEntrySensor() {
        return !intakeSensor.get();
      }
    
    public boolean getShooterSensor() {
      return !shootSensor.get();
    }

}
