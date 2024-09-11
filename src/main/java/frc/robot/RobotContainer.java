// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.sql.Driver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.manipulators.Manipulator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/maxSwerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
  final CommandXboxController driverXbox = new CommandXboxController(1);

  private final Manipulator manip = Manipulator.getInstance();
 
  private double currArmTarget = Manipulator.kARM_START_POS;
  private int intake_counter = 0;

        // Intake Command
        Command driveControls = new RunCommand(() -> {
          // PS4 Controller intake&shooter
            // if pressing intake and note is not there
            if ((driverPS4.R1().getAsBoolean() || driverXbox.rightBumper().getAsBoolean()) && manip.getNoteShooterSensor()) {
                  manip.intake(0.0);
            } else if ((driverPS4.R1().getAsBoolean() || driverXbox.rightBumper().getAsBoolean()) && !manip.getNoteEntrySensor()) { // && manip.getNoteSensor()
                  
              // if (manip.getArmPosition() < 1) {
                  this.currArmTarget = Manipulator.kARM_FLOOR_POS;
              //     }
                  manip.intake(-1.0);
                  // intake_counter++;
                  // if (intake_counter < 100) {
                  //   this.currArmTarget = Manipulator.kARM_FLOOR_POS;
                  // }
                  // else {
                  //   this.currArmTarget = Manipulator.kARM_FENDER_POS;
                  // }
                  // outtake
              } else if ((driverPS4.R1().getAsBoolean() || driverXbox.rightBumper().getAsBoolean()) && manip.getNoteEntrySensor()) {
              // if (manip.getArmPosition() < 1) {
              this.currArmTarget = Manipulator.kARM_FENDER_POS;
              // }
                manip.intake(-1.0);
              
              } else if (driverPS4.L1().getAsBoolean()||driverXbox.leftBumper().getAsBoolean()) {
                  manip.intake(0.5);
                  manip.shoot(-0.5);
                  // shoot in speaker
              } else if (driverPS4.R2().getAsBoolean()||driverXbox.rightTrigger().getAsBoolean()) {
                manip.intake(-1.0);
                manip.shoot(0.65);
                  // idle
              } else if (driverPS4.L2().getAsBoolean()||driverXbox.leftTrigger().getAsBoolean()) {
                  this.currArmTarget = Manipulator.kARM_HIGH_POS;
                  manip.intake(0.0);
                  manip.shoot(0.65);
              } else {
                  manip.intake(0.0);
                  manip.shoot(0.0);
              }

            // prevent dragging on ground
            driverPS4.R1().onFalse(new InstantCommand(() -> {
              if (manip.getArmPosition() < Manipulator.kARM_FENDER_POS)
                 this.currArmTarget = Manipulator.kARM_FENDER_POS;
              intake_counter = 0;
            }));
             driverXbox.rightBumper().onFalse(new InstantCommand(() -> {
              if (manip.getArmPosition() < Manipulator.kARM_FENDER_POS)
                 this.currArmTarget = Manipulator.kARM_FENDER_POS;
              intake_counter = 0;
            }));

            // arm
            if (driverPS4.square().getAsBoolean()||driverXbox.x().getAsBoolean()) {
              this.currArmTarget = Manipulator.kARM_AMP_POS;
            }

            if (driverPS4.triangle().getAsBoolean()||driverXbox.y().getAsBoolean()) {
              this.currArmTarget = Manipulator.kARM_HIGH_POS;
            }

            if (driverPS4.circle().getAsBoolean()||driverXbox.b().getAsBoolean()) {
              this.currArmTarget = Manipulator.kARM_FENDER_POS;
            }

            if (driverPS4.povLeft().getAsBoolean()||driverXbox.povRight().getAsBoolean()) { 
              this.currArmTarget = Manipulator.kARM_FLOOR_POS;
            }

            if (driverPS4.povRight().getAsBoolean()||driverXbox.povUpRight().getAsBoolean()) { 
              manip.resetArmEncoder();
            }

            if (driverPS4.povUp().getAsBoolean() || driverXbox.povUp().getAsBoolean()) {
                manip.moveArm(-0.5); // Up
                this.currArmTarget = manip.getArmPosition();
            } else if (driverPS4.povDown().getAsBoolean() || driverXbox.povDown().getAsBoolean()) {
                manip.moveArm(0.5); // Down
                this.currArmTarget = manip.getArmPosition();
            } else {
                manip.armToPosition(this.currArmTarget);
                // System.out.println("Moving arm");
            }
          //Xbox Controller
            // driverXbox.rightBumper().onFalse(new InstantCommand(() -> {
            //   if (manip.getArmPosition() < Manipulator.kARM_FENDER_POS)
            //      this.currArmTarget = Manipulator.kARM_FENDER_POS;
            //   intake_counter = 0;
            // }));
            // if (driverPS4.triangle().getAsBoolean()) {
            //   manip.armToPosition(SmartDashboard.getNumber("Arm Target", 1));
            // }

            SmartDashboard.putNumber("Arm", manip.getArmPosition());
            SmartDashboard.putNumber("Arm Target", this.currArmTarget);
            SmartDashboard.putBoolean("Sensor", manip.getNoteEntrySensor());
            SmartDashboard.putBoolean("Sensor 2", manip.getNoteShooterSensor());


          });

        Command drive = driveControls.repeatedly();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
      //uses PS4
      AbsoluteDriveAdv closedAbsoluteDriveAdvP = new AbsoluteDriveAdv(drivebase,
                                                                    () -> -MathUtil.applyDeadband(driverPS4.getLeftY(),
                                                                                                  OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(driverPS4.getLeftX(),
                                                                                                  OperatorConstants.LEFT_X_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(driverPS4.getRightX(),
                                                                                                  OperatorConstants.RIGHT_X_DEADBAND),
                                                                    driverPS4.getHID()::getTriangleButtonPressed,
                                                                    driverPS4.getHID()::getCrossButtonPressed,
                                                                    driverPS4.getHID()::getSquareButtonPressed,
                                                                    driverPS4.getHID()::getCircleButtonPressed);                                         

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRightX(),
        () -> driverPS4.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverPS4.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));

        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // driverPS4.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverPS4.circle().whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    // driverPS4.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

     

        // Button bindings
        // driverPS4.cross().onTrue(intakeCommand);
        // driverPS4.square().onTrue(intakeCommand); // Assuming the same command for Square button
        // driverPS4.square().whileTrue(intakeCommand); // Assuming the same command for Square button
        // driverPS4.R2().onTrue(shooterCommand);
        // armControlCommand.schedule();

        // intakeCommand.schedule();
        // shooterCommand.schedule();
        // armControlCommand.schedule();
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }
;
  public void runAuton()
  {
    System.out.println("running");
    CommandScheduler.getInstance().cancelAll();

    // Timer.delay(0.7);
    // Command auto = new RunCommand(() -> { 
    //   drivebase.driveCommand(
    //     () -> -0.8,
    //     () -> 0,
    //     () -> 0);

    //     Timer.delay(2.5);

    //     drivebase.driveCommand(
    //     () -> 0,
    //     () -> 0,
    //     () -> 0);
    // });

    // move arm up
    Command prepToShootPreload = new RunCommand(() -> {
      manip.armToPosition(Manipulator.kARM_FENDER_POS);
    });

    Command armTask = prepToShootPreload.repeatedly();

    // Command speedFlywheel = new RunCommand(() -> {
    //   // speed up shooter
    //   manip.shoot(0.65);
    // });

    // Command runIntake = new RunCommand(() -> {
    //   manip.intake(-1);
    // });

    
    armTask.schedule();

    // if (manip.getArmPosition() >= 0.12) {
    //   speedFlywheel.schedule();
    //   runIntake.schedule();
    // }

    // time to keep shooter on for
    // Timer.delay(2);

    // run intake to shoot
    // manip.intake(-1);
  
    // auto.schedule();
    drivebase.driveCommand(
        () -> -0.8,
        () -> 0,
        () -> 0).schedule();
  };

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
