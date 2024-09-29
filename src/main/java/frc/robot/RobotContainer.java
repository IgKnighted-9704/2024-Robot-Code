// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.OffenseSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandPS4Controller driverPS4 = new CommandPS4Controller(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  private final OffenseSubsystem offense = new OffenseSubsystem(Constants.SubsystemContants.IntakeId, Constants.SubsystemContants.FlyA, Constants.SubsystemContants.FlyB, Constants.SubsystemContants.ArmID,Constants.SubsystemContants.intakeSensor,Constants.SubsystemContants.shootSensor);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
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
        () -> driverPS4.getRightX() * 0.5);


    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRawAxis(2));





    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
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
                                  //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                                  // driverXbox.b().whileTrue(
                                  //     Commands.deferredProxy(() -> drivebase.driveToPose(
                                  //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                                  //                           ));
                                  // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
                                  // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    //intake
      Command intakeCommand = new ParallelCommandGroup(
        new RunCommand(()-> {
        offense.intake(0.5);
      }),
        new InstantCommand(() -> {
          offense.raiseArm2Position(Constants.ArmSetupConstants.ARM_IntakePos);
        })
      );
      Command autoOuttake = new SequentialCommandGroup(
        new RunCommand(() -> {
          offense.outtake(0.5, true);
        }),
        new WaitCommand(0.2),
        new RunCommand(() -> {
          offense.stopOuttakeAIntake();
        })
    );
    driverPS4.R1().whileTrue(intakeCommand).onFalse(autoOuttake);
    driverXbox.rightBumper().whileTrue(intakeCommand).onFalse(autoOuttake);

    //outtake
    driverPS4.L1().whileTrue(new RunCommand(() -> offense.outtake(0.5,false), offense)).onFalse(new RunCommand(() -> offense.intake(0.0), offense));
    driverXbox.leftBumper().whileTrue(new RunCommand(() -> offense.outtake(0.5,false), offense)).onFalse(new RunCommand(() -> offense.intake(0.0), offense));

    //rev flywheels
    driverPS4.L2().whileTrue(new RunCommand(() -> offense.shootConsistently(6500), offense)).onFalse(new RunCommand(() -> offense.shootConsistently(0), offense));
    driverXbox.leftTrigger().whileTrue(new RunCommand(() -> offense.shootConsistently(6500), offense)).onFalse(new RunCommand(() -> offense.shootConsistently(0.0), offense));

    //shoot
      Command shootCommand = new ParallelCommandGroup(
        new RunCommand(() -> {
        offense.shoot(0.75);
      }),
        new InstantCommand(() ->{
          offense.intake(0.5);
        })
      );

    driverPS4.R2().whileTrue(shootCommand).onFalse(new RunCommand(() -> offense.shootConsistently(0)));
    driverXbox.rightTrigger().whileTrue(shootCommand).onFalse(new RunCommand(() -> offense.shootConsistently(0)));
    //amp pos
    driverPS4.triangle().onTrue(new InstantCommand(() -> offense.raiseArm2Position(Constants.ArmSetupConstants.ARM_AmpPos), offense));
    driverXbox.y().onTrue(new InstantCommand(() -> offense.raiseArm2Position(Constants.ArmSetupConstants.ARM_AmpPos), offense));

    //speaker pos
    driverPS4.square().onTrue(new InstantCommand(() -> offense.raiseArm2Position(Constants.ArmSetupConstants.ARM_SpeakerPos), offense));
    driverXbox.x().onTrue(new InstantCommand(() -> offense.raiseArm2Position(Constants.ArmSetupConstants.ARM_SpeakerPos), offense));

    //raise arm
    driverPS4.povUp().whileTrue(new RunCommand(() -> offense.raiseArm(0.5), offense)).onFalse(new RunCommand(() -> offense.raiseArm(0.0), offense));
    driverXbox.povUp().whileTrue(new RunCommand(() -> offense.raiseArm(0.5), offense)).onFalse(new RunCommand(() -> offense.raiseArm(0.0), offense));
    //lower arm
    driverPS4.povUp().whileTrue(new RunCommand(() -> offense.raiseArm(-0.5), offense)).onFalse(new RunCommand(() -> offense.raiseArm(0.0), offense));
    driverXbox.povUp().whileTrue(new RunCommand(() -> offense.raiseArm(-0.5), offense)).onFalse(new RunCommand(() -> offense.raiseArm(0.0), offense));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    
    //autonomous commands

    //parallel shoot command
    Command shootCommand = new ParallelCommandGroup(
      new RunCommand(() -> {
      offense.shoot(0.75);
    }),
      new InstantCommand(() ->{
       offense.intake(0.5);        
      }));

    //parallel stop command
    Command stopshootCommand = new ParallelCommandGroup(
      new RunCommand(() -> {
      offense.shoot(0);
    }),
      new InstantCommand(() ->{
       offense.intake(0);        
      }));

    //drives to left most note
    Command autonLMNote = new SequentialCommandGroup(
      new RunCommand(() -> {
        drivebase.getAutonomousCommand("LN - Path");
      }),
      new RunCommand(() -> {
        offense.intake(0.5);
      }),
      new WaitCommand(2),
      new InstantCommand(() -> {
        offense.intake(0);
      }),
      new RunCommand(() -> {
        offense.outtake(0.25, false);
      }),
      new InstantCommand(() -> {
        offense.outtake(0, true);
      }),
      new WaitCommand(0.2),
      new RunCommand(() -> {
        drivebase.getAutonomousCommand("Shoot - Path");
    }),
      shootCommand,
      new WaitCommand(2)
      ,stopshootCommand);

      return autonLMNote;
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
