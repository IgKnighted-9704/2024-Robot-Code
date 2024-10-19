// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Initialize driver and auxiliary controllers
  final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
  final CommandXboxController auxXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  
  // Initialize the drivebase subsystem with configuration file
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/maxSwerve"));

  // Initialize the arm and shooter subsystems
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(armSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Define an autonomous shooting command sequence
    Command autonShoot = new SequentialCommandGroup(
        new InstantCommand(() -> {
            armSubsystem.moveToShoot();
        }),
        new InstantCommand(() -> {
            shooterSubsystem.spinUpShooter();
        }),
        new WaitCommand(2.0),  // Wait for 2 seconds to allow shooter to spin up
        new InstantCommand(() -> {
            shooterSubsystem.shootInSpeaker();
        }),
        new WaitCommand(1.0),  // Wait for 1 second to shoot
        new InstantCommand(() -> {
            shooterSubsystem.stopShooter();
        }));

    // Register the autonomous command with a name
    NamedCommands.registerCommand("autonShoot", autonShoot);
    
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
    // Left stick controls translation
    // Right stick controls the rotational velocity 
    // Buttons are quick rotation positions to different ways to face
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

    // Define drive commands based on controller inputs
    // Field-oriented drive with direct angle control
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRightX(),
        () -> driverPS4.getRightY());

    // Field-oriented drive with angular velocity control
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverPS4.getRightX());

    // Simulation mode drive command
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRawAxis(2));

    // Set the default drive command based on whether the robot is in simulation
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
    // Zero the gyro when the cross button is pressed on PS4 or X button on Xbox
    driverPS4.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    auxXbox.x().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Intake controls
    driverPS4.L2().whileTrue(new RunCommand(() -> shooterSubsystem.intake(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopIntake, shooterSubsystem));
    auxXbox.rightBumper().whileTrue(new RunCommand(() -> shooterSubsystem.intake(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopIntake, shooterSubsystem));

    // Additional intake behavior with sensor outtake
    driverPS4.L2().whileTrue(new RunCommand(() -> shooterSubsystem.intake(), shooterSubsystem))
        .onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> {
               shooterSubsystem.sensorOuttake();
            }),
            new WaitCommand(0.035),  // Wait for 0.035 seconds
            new InstantCommand(() -> {
                shooterSubsystem.stopIntake();
            })));

    auxXbox.leftTrigger().whileTrue(new RunCommand(() -> shooterSubsystem.intake(), shooterSubsystem))
        .onFalse(new SequentialCommandGroup(
            new InstantCommand(() -> {
               shooterSubsystem.sensorOuttake();
            }),
            new WaitCommand(0.035),  // Wait for 0.035 seconds
            new InstantCommand(() -> {
                shooterSubsystem.stopIntake();
            })));
            
    // Outtake controls
    driverPS4.L1().whileTrue(new RunCommand(() -> shooterSubsystem.outtake(), shooterSubsystem))
      .onFalse(new InstantCommand(shooterSubsystem::stopIntake, shooterSubsystem));
    auxXbox.leftBumper().whileTrue(new RunCommand(() -> shooterSubsystem.outtake(), shooterSubsystem))
      .onFalse(new InstantCommand(shooterSubsystem::stopIntake, shooterSubsystem));  

    // Spin up shooter
    driverPS4.R1().whileTrue(new RunCommand(() -> shooterSubsystem.spinUpShooter(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    auxXbox.rightBumper().whileTrue(new RunCommand(() -> shooterSubsystem.spinUpShooter(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    
    // Shoot controls
    driverPS4.R2().whileTrue(new RunCommand(() -> shooterSubsystem.shootInSpeaker(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    auxXbox.rightTrigger().whileTrue(new RunCommand(() -> shooterSubsystem.shootInSpeaker(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    
    // Arm positioning controls
    driverPS4.square().whileTrue(new RunCommand(() -> shooterSubsystem.spinUpFeed(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    driverPS4.circle().onTrue(new InstantCommand(() -> armSubsystem.moveToShoot()));
    driverPS4.triangle().onTrue(new InstantCommand(() -> armSubsystem.moveToAmp()));

    auxXbox.b().whileTrue(new RunCommand(() -> shooterSubsystem.spinUpFeed(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    auxXbox.a().onTrue(new InstantCommand(()-> armSubsystem.moveToShoot()));
    auxXbox.y().onTrue(new InstantCommand(() -> armSubsystem.moveToAmp()));

    // Arm encoder reset and manual adjustments
    driverPS4.povRight().onTrue(new InstantCommand(() -> armSubsystem.resetArmEncoder()));
    driverPS4.povUp().onTrue(new InstantCommand(() -> armSubsystem.setSetpoint(armSubsystem.getMeasurement() + 0.05)));
    driverPS4.povDown().onTrue(new InstantCommand(() -> armSubsystem.setSetpoint(armSubsystem.getMeasurement() - 0.05)));

    auxXbox.povRight().onTrue(new InstantCommand(() -> armSubsystem.resetArmEncoder()));
    auxXbox.povUp().onTrue(new InstantCommand(() -> armSubsystem.setSetpoint(armSubsystem.getMeasurement() + 0.05)));
    auxXbox.povDown().onTrue(new InstantCommand(() -> armSubsystem.setSetpoint(armSubsystem.getMeasurement() - 0.05)));

    // Additional commands (currently commented out)
    // driverPS4.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverPS4.circle().whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    // driverPS4.triangle().whileTrue(drivebase.aimAtSpeaker(2));
    // driverPS4.cross().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the autonomous command named "Test Auton"
    return drivebase.getAutonomousCommand("Test Auton");
    // Alternatively, use a different autonomous command
    // return drivebase.getAutonomousCommand("Midline Disrupt");
  }

  // Alternative autonomous command sequence (currently commented out)
  // public Command getAutonomousCommand() {
  //   // Create a new command for the autonomous period
  //   Command autonomousCommand = new SequentialCommandGroup(
  //       new InstantCommand(() -> {
  //           armSubsystem.moveToShoot();
  //       }),
  //       new InstantCommand(() -> {
  //           shooterSubsystem.spinUpShooter();
  //       }),
  //       new WaitCommand(2.0),  // Wait for 2 seconds
  //       new InstantCommand(() -> {
  //           shooterSubsystem.shootInSpeaker();
  //       }),
  //       new WaitCommand(1.0),  // Wait for 1 second
  //       new InstantCommand(() -> {
  //           shooterSubsystem.stopShooter();
  //       }),
  //       new InstantCommand(() -> {
  //           drivebase.driveCommand(
  //       () -> -0.8,
  //       () -> 0,
  //       () -> 0).schedule();
  //       }),
  //       new WaitCommand(1.4),  // Wait for 1.4 seconds
  //       new InstantCommand(() -> {
  //           drivebase.driveCommand(
  //       () -> 0,
  //       () -> 0,
  //       () -> 0).schedule();
  //       }));        

  //   return autonomousCommand;
  // }

  /**
   * Sets the drive mode of the robot (method currently empty).
   */
  public void setDriveMode()
  {
    // drivebase.setDefaultCommand();
  }

  /**
   * Resets the gyro heading to align with the red alliance orientation.
   */
  public void resetToRed() {
    drivebase.zeroGyroWithAlliance();
  }

  /**
   * Sets the motor brake mode for the drivebase.
   *
   * @param brake True to enable brake mode, false to disable.
   */
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
