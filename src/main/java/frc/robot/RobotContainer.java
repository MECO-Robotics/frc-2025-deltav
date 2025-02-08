// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.HandoffCommandReverse;
import frc.robot.commands.arm.SetPointControlCommand;
import frc.robot.commands.indexer.IndexingCommand;
import frc.robot.commands.intake.NoAutomationIntakieCommand;
import frc.robot.commands.leds.FlashOnceCommand;
import frc.robot.commands.leds.LedDefaultCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.swervedrive.auto.ShootInPlaceAuto;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        // creates variable for controllerSubsystem
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final IndexingSubsystem indexingSubsystem = new IndexingSubsystem();
        private final LEDSubsystem led = new LEDSubsystem();

        // The robot's subsystems and commands are defined here...
        //private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        //"swerve/neo"));

        DoubleSupplier armAimAngle = () -> SmartDashboard.getNumber("Arm Aim Angle", 0);

        XboxController pilotController = new XboxController(0);
        CommandXboxController pilotCommandController = new CommandXboxController(0);
        XboxController coPilotController = new XboxController(1);
        CommandXboxController coPilotCommandController = new CommandXboxController(1);

        // Command aimCommand = new AbsoluteFieldDrive(drivebase,
        // () -> MathUtil.applyDeadband(-pilotController.getLeftY(),
        // OperatorConstants.LEFT_Y_DEADBAND),
        // () -> MathUtil.applyDeadband(-pilotController.getLeftX(),
        // OperatorConstants.LEFT_X_DEADBAND),
        // () -> 2 * drivebase.angletoSpeaker().getRotations());
        // new SetPointControlCommand(armSubsystem, armAimAngle));
        // //new ShooterCommand(shooterSubsystem,
        // Constants.Shooter.Presets.kLeftSpeaker,
        // //Constants.Shooter.Presets.kRightSpeaker)); // TODO create aiming
        // // equation

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

        public RobotContainer() {
                SmartDashboard.putNumber("Arm Aim Angle", 0);

                // Commands for Pathplanner
                NamedCommands.registerCommand("Shoot", new ShooterCommand(shooterSubsystem,
                                Constants.Shooter.Presets.kLeftSpeaker, Constants.Shooter.Presets.kRightSpeaker));
                NamedCommands.registerCommand("Intake",
                                new ParallelCommandGroup(new HandoffCommand(indexingSubsystem, intakeSubsystem, led),
                                                new PrintCommand("HandOff Command running")));
                NamedCommands.registerCommand("RunIndexer", new IndexingCommand(indexingSubsystem, 12));
                NamedCommands.registerCommand("StopIndexer", new IndexingCommand(indexingSubsystem, 0));
                // NamedCommands.registerCommand("Arm",
                // new SetPointControlCommand(armSubsystem,
                // Constants.Arm.SetPointPositions.kBeamFlatPosition));
                NamedCommands.registerCommand("Down", new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kStowPosition));
                NamedCommands.registerCommand("amp",
                                new SetPointControlCommand(armSubsystem, Constants.Arm.SetPointPositions.kAmpPosition));
                // NamedCommands.registerCommand("Sniper", aimCommand);
                NamedCommands.registerCommand("eject",
                                new SequentialCommandGroup(new IndexingCommand(indexingSubsystem, 12),
                                                new WaitCommand(.5), new IndexingCommand(indexingSubsystem, 0)));

                // Auto selection choices
                SmartDashboard.putData("PathPlannerAuto", autoCommandChoice);
                // autoCommandChoice.addOption("7 note auto", "7 note auto");

                // Anything
                autoCommandChoice.addOption("Shoot in place", "ShootInPlaceAuto");

                // Blue aliance
                autoCommandChoice.addOption("4 note", "4 note");
                autoCommandChoice.addOption("blue sniper", "blue sniper");
                // Red aliance
                autoCommandChoice.addOption("red note", "red note");

                // mason sutpp

                // These old autos but I dont want to touch these and break auto choice thingy
                // SmartDashboard.putData("4 note(3 close) middle auto", autoCommandChoice);
                // SmartDashboard.putData("4 note(3 close) bottom auto", autoCommandChoice);

               // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                // Applies deadbands and inverts controls because joysticks
                                // are back-right positive while robot
                                // controls are front-left positive
                              //  () -> MathUtil.applyDeadband(-pilotController.getLeftY(),
                              //                  OperatorConstants.LEFT_Y_DEADBAND),
                               // () -> MathUtil.applyDeadband(-pilotController.getLeftX(),
                               //                 OperatorConstants.LEFT_X_DEADBAND),
                               // () -> -pilotController.getRightX(),
                                //() -> -pilotController.getRightY());

            //    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
             //                   () -> MathUtil.applyDeadband(pilotController.getLeftY(),
             //                                   OperatorConstants.LEFT_Y_DEADBAND),
             //                   () -> MathUtil.applyDeadband(pilotController.getLeftX(),
             //                                   OperatorConstants.LEFT_X_DEADBAND),
             //                   () -> pilotController.getRawAxis(2));
//
               // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
              //                  () -> MathUtil.applyDeadband(pilotController.getLeftY(),
              //                                  OperatorConstants.LEFT_Y_DEADBAND),
               //                 () -> MathUtil.applyDeadband(pilotController.getLeftX(),
              //                                  OperatorConstants.LEFT_X_DEADBAND),
              //                  () -> MathUtil.applyDeadband(pilotController.getRightX(),
              //                                  OperatorConstants.RIGHT_X_DEADBAND),
             //                   pilotController::getYButtonPressed,
              //                  pilotController::getAButtonPressed,
              //                  pilotController::getXButtonPressed,
              //                  pilotController::getBButtonPressed);

              /*   TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                () -> MathUtil.applyDeadband(-pilotController.getLeftY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(-pilotController.getLeftX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> pilotController.getRawAxis(2), () -> true);

                TeleopDrive closedFieldRel = new TeleopDrive(
                                drivebase,
                                () -> MathUtil.applyDeadband(-pilotCommandController.getLeftY(),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(-pilotCommandController.getLeftX(),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> -pilotController.getRightX(), () -> true);
*/
                // ManualArmControlCommand manualArm = new ManualArmControlCommand(armSubsystem,
                // () -> MathUtil.applyDeadband(coPilotController.getRightY() * -12, 0.01));

                // Configure the trigger bindingss
                configureBindings();

            /*     drivebase.setDefaultCommand(closedFieldRel);

                intakeSubsystem.setDefaultCommand(
                                new NoAutomationIntakieCommand(intakeSubsystem,
                                                () -> pilotController.getRightTriggerAxis() * -12));
                indexingSubsystem.setDefaultCommand(new IndexingCommand(indexingSubsystem,
                                () -> (pilotController.getLeftTriggerAxis() + coPilotController.getLeftTriggerAxis())
                                                * 12));

                led.setDefaultCommand(new LedDefaultCommand(led).ignoringDisable(true));

                // armSubsystem.setDefaultCommand(new SetPointControlCommand(armSubsystem, () ->
                // SmartDashboard.getNumber("Arm Setpoint", 0)));
*/
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
              

                // ----------------------------
                // Pilot Controls
                // ----------------------------
                pilotCommandController.b().whileTrue(new NoAutomationIntakieCommand(intakeSubsystem, () -> -12));
                pilotCommandController.rightBumper()
                                .whileTrue(
                                                new ParallelCommandGroup(
                                                                new ShooterCommand(shooterSubsystem, -2000, -2000),
                                                                new IndexingCommand(indexingSubsystem, () -> -6)));
                // new SequentialCommandGroup(
                // new HandoffCommand
                // (indexingSubsystem, intakeSubsystem, led,
                // pilotController, coPilotController),
                // new FlashOnceCommand(led, Color.kGreen))
                pilotCommandController.leftBumper()
                                .whileTrue(new SequentialCommandGroup(
                                                new HandoffCommandReverse(indexingSubsystem, intakeSubsystem, led,
                                                                pilotController, coPilotController),
                                                new FlashOnceCommand(led, Color.kGreen)));

                pilotCommandController.leftBumper().onTrue(new HandoffCommand(indexingSubsystem, intakeSubsystem, led));

           //     pilotCommandController.y().onTrue((new InstantCommand(drivebase::zeroGyro)));

                pilotCommandController.povDown().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kStowPosition));

                pilotCommandController.a().onTrue(new InstantCommand(shooterSubsystem::disable));

                pilotCommandController.povDown().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kStowPosition));

                pilotCommandController.povRight().onTrue(
                                new SetPointControlCommand(armSubsystem,
                                                Constants.Arm.SetPointPositions.kShootFlatPosition));

                pilotCommandController.povUp().onTrue(
                                new SetPointControlCommand(armSubsystem, Constants.Arm.SetPointPositions.kAmpPosition));

                pilotCommandController.povRight().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kHumanPosition));
                pilotCommandController.povLeft().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kShootFlatPosition));
                pilotCommandController.x()
                                .whileTrue(new ShooterCommand(shooterSubsystem, Constants.Shooter.Presets.kLeftSpeaker,
                                                Constants.Shooter.Presets.kRightSpeaker));
                // pilotCommandController.x().whileTrue(aimCommand);

                // ----------------------------
                // Co-Pilot Controls
                // ----------------------------
                coPilotCommandController.x()
                                .whileTrue(new ShooterCommand(shooterSubsystem, Constants.Shooter.Presets.kLeftSpeaker,
                                                Constants.Shooter.Presets.kRightSpeaker));

                coPilotCommandController.a().onTrue(new InstantCommand(shooterSubsystem::disable));

                coPilotCommandController.povDown().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kStowPosition));
                coPilotCommandController.povRight().onTrue(
                                new SetPointControlCommand(armSubsystem,
                                                Constants.Arm.SetPointPositions.kShootFlatPosition));

                coPilotCommandController.povUp().onTrue(
                                new SetPointControlCommand(armSubsystem, Constants.Arm.SetPointPositions.kAmpPosition));

                coPilotCommandController.povRight().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kHumanPosition));
                coPilotCommandController.povLeft().onTrue(new SetPointControlCommand(armSubsystem,
                                Constants.Arm.SetPointPositions.kShootFlatPosition));

                /*
                 * coPilotCommandController.rightStick()
                 * .whileTrue(new ManualArmControlCommand(armSubsystem,
                 * () -> coPilotCommandController.getRightY() > 0
                 * ? -coPilotCommandController.getRightY()
                 * : 0));
                 */

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                if (autoCommandChoice != null && autoCommandChoice.getSelected() != null) {
                        if (autoCommandChoice.getSelected() == "ShootInPlaceAuto") {
                                return new ShootInPlaceAuto(shooterSubsystem, indexingSubsystem);
                        } else {
                                return new PathPlannerAuto(autoCommandChoice.getSelected());
                        }
                }

                return null;

                // return new PathPlannerAuto("arm move");
        }

        public void setDriveMode() {
                // drivebase.setDefaultCommand();
        }

       public void setMotorBrake(boolean brake) {
            //    drivebase.setMotorBrake(brake);
        }

}
