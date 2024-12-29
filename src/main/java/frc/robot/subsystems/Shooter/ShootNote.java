package frc.robot.subsystems.Shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {

  private ShooterSubsystem shooter;

  /** Creates a new IntakeNote. */
  public ShootNote(ShooterSubsystem shooter){

    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //shooter.autoSHooterCommand(4000);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.stop();
  }

 // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
