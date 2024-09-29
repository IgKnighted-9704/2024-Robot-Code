package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OffenseSubsystem extends SubsystemBase {
    private final CANSparkMax intake;
    private final CANSparkMax shootMotor1;
    private final CANSparkMax shootMotor2;
    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final PIDController pidControllerKarm = new PIDController(Constants.SubsystemContants.kP_Arm,Constants.SubsystemContants.kI_Arm,Constants.SubsystemContants.kD_Arm);
    private final PIDController shooterPID = new PIDController(0.0001, 0, 0);
    private final SimpleMotorFeedforward shootFeed = new SimpleMotorFeedforward(0, 0, 0);
    private final DigitalInput intakeSensor;
    private final DigitalInput shootSensor;


    public OffenseSubsystem(int IntakeId, int SM1, int SM2,int ArmId, int intakeSensor, int shootSensor){
        intake = new CANSparkMax(IntakeId, CANSparkLowLevel.MotorType.kBrushless);
        shootMotor1 = new CANSparkMax(SM1, CANSparkLowLevel.MotorType.kBrushless);
        shootMotor2 = new CANSparkMax(SM2, CANSparkLowLevel.MotorType.kBrushless);
        shootMotor2.setInverted(true);
        armMotor = new CANSparkMax(ArmId,CANSparkLowLevel.MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        this.intakeSensor = new DigitalInput(intakeSensor);
        this.shootSensor = new DigitalInput(shootSensor);

        raiseArm2Position(Constants.ArmSetupConstants.ARM_DefaultPos);
    }

    public void shootConsistently(double RPM){
        shootMotor1.setVoltage(shooterPID.calculate(getShooterSpeed(), RPM) + shootFeed.calculate(RPM));
        shootMotor2.setVoltage(shooterPID.calculate(getShooterSpeed(), RPM) + shootFeed.calculate(RPM));
    }
        //private method to run consistent speed on shooter
        private double getShooterSpeed(){
            return(shootMotor1.getEncoder().getVelocity() + shootMotor2.getEncoder().getVelocity())/2;
        }

        //private methods for intake/shooter
            private void intakeNote(double speed){
                intake.set(speed);
            }
            private void shootNote(double speed){
                shootMotor1.set(speed);
                shootMotor2.set(speed);
            }

    public void intake(double speed){
        if(getEntrySensor() && getShooterSensor()){
            intakeNote(0);
        } else {
            intake.set(speed);
        }
    }

    public void shoot(double speed){
        shootNote(speed);
    }

    public void outtake(double speed, boolean auto){
        if(auto){
            if(getShooterSensor()){
                intake.set(speed);
            }
        } else {
            shoot(-speed);
            intake.set(speed);
        }
    }

    public void stopOuttakeAIntake(){
        intake(0);
    }

    public void raiseArm2Position(double targetPos){
        pidControllerKarm.setSetpoint(targetPos);
        double power = pidControllerKarm.calculate(armEncoder.getPosition());
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
