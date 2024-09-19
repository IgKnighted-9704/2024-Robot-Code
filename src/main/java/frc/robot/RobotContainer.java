// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
  final CommandXboxController auxXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/maxSwerve"));

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
   private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(armSubsystem);

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
    auxXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // intake
    driverPS4.R1().whileTrue(new RunCommand(() -> shooterSubsystem.intake(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopIntake, shooterSubsystem));
    
    // outtake
    driverPS4.L1().whileTrue(new RunCommand(() -> shooterSubsystem.outtake(), shooterSubsystem))
      .onFalse(new InstantCommand(shooterSubsystem::stopIntake, shooterSubsystem));

    // spin up shooter
    driverPS4.L2().whileTrue(new RunCommand(() -> shooterSubsystem.spinUpShooter(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));

    // shoot
    driverPS4.R2().whileTrue(new RunCommand(() -> shooterSubsystem.shootInSpeaker(), shooterSubsystem))
        .onFalse(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
    
    // Arm positioning with Triangle (mapped from old control)
    driverPS4.triangle().onTrue(new InstantCommand(() -> armSubsystem.moveToShoot()));
    driverPS4.circle().onTrue(new InstantCommand(() -> armSubsystem.moveToShoot()));

    driverPS4.square().onTrue(new InstantCommand(() -> armSubsystem.moveToAmp()));


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
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
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
