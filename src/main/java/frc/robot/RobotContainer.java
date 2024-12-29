// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.IntakeNote;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.OutakeNote;
import frc.robot.subsystems.Shooter.ShootNote;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import com.ctre.phoenix6.Utils;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstantsFollower;
import frc.robot.Constants.ShooterConstantsLeader;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.FeedNote;
import frc.robot.subsystems.Intake.IntakeNote;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.OutakeNote;
import frc.robot.subsystems.Shooter.ShootNote;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final Arm m_armSubsystem = new Arm();
    //private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final IntakeNote m_intakenoteSubsystem = new IntakeNote(m_intakeSubsystem);
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public void teleopPeriodic() {
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
            //main commands\\

    driver.leftBumper().onTrue(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(-2.5)))); //intake
    driver.leftBumper().onFalse(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(-2.5))));

    operator.x().onTrue(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(43.5)))); //podium
    operator.x().onFalse(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(-4))));

    driver.leftTrigger().onTrue(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(75)))); //traveling
    driver.leftTrigger().onFalse(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(-2.5))));

    operator.a().onTrue(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(116)))); //amp
    //operator.a().onFalse(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(-2.5))));

    driver.a().onTrue(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(12.5)))); //sub
    driver.a().onFalse(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(-2.5))));
    
    //shooter commands to arm postitions\\
    
    driver.leftBumper().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop())); //intake
    operator.x().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop())); //podium
    driver.leftTrigger().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop())); //traveling
    //operator.a().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop())); //amp
    driver.a().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop())); //sub

    //driver.rightTrigger().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));

    
    operator.rightTrigger().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
    operator.leftTrigger().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(-16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
    driver.y().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(-16000))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));

    //operator.leftTrigger().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(4000))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));


    driver.rightBumper().whileTrue(new IntakeNote(m_intakeSubsystem));
    //driver.rightTrigger().whileTrue(new FeedNote(m_intakeSubsystem));
    driver.y().whileTrue(new OutakeNote(m_intakeSubsystem));
    operator.leftTrigger().whileTrue(new OutakeNote(m_intakeSubsystem));
    driver.x().whileTrue( new RunCommand(()-> m_shooterSubsystem.enableIntake(m_intakeSubsystem)));
  }//operator.a().whileTrue(runOnce(() -> m_intakeSubsystem.setVelocity(1000))).onFalse(runOnce(() -> m_intakeSubsystem.stop()));

  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    NamedCommands.registerCommand("IntakeNote" , new IntakeNote(m_intakeSubsystem));
    NamedCommands.registerCommand("ShootNote", new ShootNote(m_shooterSubsystem));
    NamedCommands.registerCommand("ShootNote2", Commands.sequence(runOnce(() -> m_armSubsystem.setAngle(Rotation2d.fromDegrees(12.5))),  runOnce(() -> m_shooterSubsystem.setVelocity(16000)), Commands.waitSeconds(4.0), new IntakeNote(m_intakeSubsystem), Commands.waitSeconds(1.5), runOnce(() -> m_shooterSubsystem.stop())));
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  public Command getAutonomousCommand() {
    return  autoChooser.getSelected();
  }
}
