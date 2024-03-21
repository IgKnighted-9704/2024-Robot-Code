// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private final Manipulator manip = Manipulator.getInstance();

  private double currArmTarget = 0.0;

  private boolean shooterOn = false;

        // Intake Command
        Command driveControls = new RunCommand(() -> {

          // if pressing intake and note is not there
          if (driverPS4.R1().getAsBoolean() && manip.getNoteSensor()) {
                manip.intake(0.375);
                this.currArmTarget = Manipulator.kARM_FLOOR_POS;
                // outtake
            } else if (driverPS4.L1().getAsBoolean()) {
                manip.intake(-1.0);
                manip.shoot(-0.8);
                // shoot in speaker
            } else if (driverPS4.R2().getAsBoolean()) {
              manip.intake(1.0);
              manip.shoot(1.0);
                // idle
            } else if (!this.shooterOn) {
                manip.intake(0.0);
                manip.shoot(0.0);
            } else {
                manip.intake(0.0);
                manip.shoot(-0.8);
            }

            // prevent dragging on ground
            driverPS4.R1().onFalse(new InstantCommand(() -> {
              this.currArmTarget = Manipulator.kARM_FENDER_POS;
            }));

            if (driverPS4.L2().getAsBoolean()) {
              shooterOn = !shooterOn;
            }

            // arm
            if (driverPS4.square().getAsBoolean()) {
              this.currArmTarget = Manipulator.kARM_AMP_POS;
            }

            if (driverPS4.circle().getAsBoolean()) {
              this.currArmTarget = Manipulator.kARM_HIGH_POS;
            }

            if (driverPS4.triangle().getAsBoolean()) {
              this.currArmTarget = Manipulator.kARM_START_POS;
            }

            if (driverPS4.povUp().getAsBoolean()) {
                manip.moveArm(0.5); // Up
                this.currArmTarget = manip.getArmPosition();
            } else if (driverPS4.povDown().getAsBoolean()) {
                manip.moveArm(-0.5); // Down
                this.currArmTarget = manip.getArmPosition();
            } else {
                manip.armToPosition(currArmTarget);
            }

            SmartDashboard.putNumber("Arm", manip.getArmPosition());

          });

        Command drive = driveControls.repeatedly();

        // // Intake Command
        // Command intakeCommand = new InstantCommand(() -> {
        //     if (driverPS4.R1().getAsBoolean() && manip.getNoteSensor()) {
        //         manip.intake(0.375);
        //         if (driverPS4.getR2Axis() < 0.5) {
        //             this.currArmTarget = Manipulator.kARM_FLOOR_POS;
        //         }
        //     } else if (driverPS4.L1().getAsBoolean()) {
        //         manip.intake(-1.0);
        //         manip.shoot(-0.25);
        //     } else {
        //         manip.intake(0.0);
        //         manip.shoot(0.0);
        //     }

        //     if (driverPS4.R1().getAsBoolean() && !manip.getNoteSensor()) {
        //         // Perform rumble action
        //     }
        //     // Reset arm target on bumper release
        //     if (driverPS4.R1().getAsBoolean() == false) {
        //         this.currArmTarget = Manipulator.kARM_FENDER_POS;
        //     }
        // });

        // // Shooter Command
        // Command shooterCommand = new InstantCommand(() -> {
        //     if (driverPS4.getR2Axis() > 0.1) {
        //         if (manip.getArmPosition() < Manipulator.kARM_START_POS) {
        //             manip.shoot(0.25);
        //         } else {
        //             // High goal shooting and vision aiming logic
        //             this.currArmTarget = Manipulator.kARM_HIGH_POS;
        //         }
        //     }

        //     if (driverPS4.getR2Axis() > 0.5) {
        //         if (manip.getArmPosition() < Manipulator.kARM_START_POS) {
        //             manip.intake(1.0);
        //             manip.shoot(0.5);
        //         } else {
        //             manip.shoot((driverPS4.getR2Axis() - 0.5) * 2);
        //         }

        //         if (driverPS4.R1().getAsBoolean()) {
        //             manip.intake(1.0);
        //         }
        //     } else {
        //         manip.shoot(0.0);
        //     }
        // });

        // // Arm Control Command
        // Command armControlCommand = new RunCommand(() -> {
        //     if (driverPS4.triangle().getAsBoolean()) {
        //         this.currArmTarget = Manipulator.kARM_AMP_POS;
        //     }

        //     if (driverPS4.povUp().getAsBoolean()) {
        //         manip.moveArm(0.5); // Up
        //         this.currArmTarget = manip.getArmPosition();
        //     } else if (driverPS4.povDown().getAsBoolean()) {
        //         manip.moveArm(-0.5); // Down
        //         this.currArmTarget = manip.getArmPosition();
        //     } else {
        //         manip.armToPosition(currArmTarget);
        //     }

        //     SmartDashboard.putNumber("Arm", manip.getArmPosition());
        // });

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

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
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedAnglularVelocity);
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

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
