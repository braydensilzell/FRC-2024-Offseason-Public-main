package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstantsLeader;
import frc.robot.Constants.ShooterConstantsFollower;
import frc.robot.Constants.ShooterConstantsLeader;
import frc.robot.subsystems.Intake.IntakeNote;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Util.ErrorCheckUtil;
import frc.robot.subsystems.Util.TalonFXFactory;
import frc.robot.subsystems.Util.ErrorCheckUtil.CommonErrorNames;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

  private TalonFX shooterTalonLeader = configureShooterLeaderTalon(TalonFXFactory.createTalon(ShooterConstantsLeader.shooterTalonLeaderID,
    ShooterConstantsLeader.shooterTalonCANBus, ShooterConstantsLeader.kShooterConfiguration));
  private TalonFX shooterTalonFollower = configureShooterFollowerTalon(TalonFXFactory.createTalon(ShooterConstantsFollower.shooterTalonFollowerID,
      ShooterConstantsFollower.shooterTalonCANBus, ShooterConstantsFollower.kShooterConfiguration));

  private TalonFX intakeBitch = new TalonFX(IntakeConstants.intakeTalonID);
  

  private Orchestra m_orchestra = new Orchestra();

  double currentSpeed;

  public ShooterSubsystem() {
    shooterTalonFollower.setControl(ShooterConstantsFollower.shooterControl);
    shooterTalonLeader.setControl(ShooterConstantsLeader.shooterControl);


    m_orchestra.addInstrument(shooterTalonLeader);
    m_orchestra.loadMusic("wonderwall.chrp");
  }

    /**
   * Set both shooter motors to the same speed
   * 
   * @param rpm rotations per minute
   */
  public void setVelocity(double rpm) {

   // shooterTalonLeader.setControl(ShooterConstants.shooterControl.withVelocity(-rps/2));
   shooterTalonFollower.setControl(ShooterConstantsFollower.shooterControl.withVelocity(rpm/60));
    shooterTalonLeader.setControl(ShooterConstantsLeader.shooterControl.withVelocity(rpm/60));
  }

  public void setShooterVoltage(double volts) {

    shooterTalonFollower.setControl(new VoltageOut(volts));
    shooterTalonLeader.setControl(new VoltageOut(volts));
  }

  public boolean isAtSetpointLeader() {
    if(shooterTalonLeader.getClosedLoopReference().getValueAsDouble() > 0) {
    return Math.abs(shooterTalonLeader.getClosedLoopError().getValue()) * 60 < ShooterConstantsLeader.shooterVelocityTolerance;
    } else {
      return false;
    }
  }

  public boolean isAtSetpointFollwer() {

    if(shooterTalonFollower.getClosedLoopReference().getValueAsDouble() > 0) {
      return Math.abs(shooterTalonFollower.getClosedLoopError().getValue()) * 60 < ShooterConstantsFollower.shooterVelocityTolerance;
      } else {
        return false;
      }
  }
  
  public void enableIntake(IntakeSubsystem m_intakeSubsystem) {
    System.out.println("running");
    double currentSpeedL = getVelocityLeader();
    double currentSpeedF = getVelocityFollower();
    System.out.println(currentSpeedL + " " + currentSpeedF);
    if(currentSpeedL > 85 && currentSpeedF > 85) {
      System.out.println("working");
      m_intakeSubsystem.Feednote();
    }

  }

  private TalonFX configureShooterFollowerTalon(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getVelocity().setUpdateFrequency(ShooterConstantsFollower.kShooterVelocityUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    return motor;
    
  }
  private TalonFX configureShooterLeaderTalon(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getVelocity().setUpdateFrequency(ShooterConstantsLeader.kShooterVelocityUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    return motor;
    
  }

  public double getVelocityFollower() {
    return shooterTalonFollower.getVelocity().getValueAsDouble();
  }

  public double getVelocityLeader() {
    return shooterTalonLeader.getVelocity().getValueAsDouble();
  }
  

  public void singWonderwall() {
    m_orchestra.play();
  }

  public void stop() {
    shooterTalonLeader.setControl(new DutyCycleOut(0));
    shooterTalonFollower.setControl(new DutyCycleOut(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter rpm", getVelocityFollower()*60);
    SmartDashboard.putNumber("shooter rpm", getVelocityLeader()*60);
    SmartDashboard.putBoolean("shooter at sp?", isAtSetpointLeader());
    SmartDashboard.putBoolean("shooter at sp?", isAtSetpointFollwer());
    
  }

      public Command runShooterCommand(double velocity) {
        return new RunCommand(()->this.setVelocity(velocity), this);
    }

    
      public void autoSHooterCommand(double velocity) {

      new RunCommand(()->this.setVelocity(velocity), this);
    
        currentSpeed = getVelocityLeader();
         currentSpeed = getVelocityFollower();

        if (velocity < currentSpeed) {
          intakeBitch.set(.8);
        } 
        
      
    }


     public Command stopShooterCommand() {
        return new InstantCommand(()->this.stop(), this);
    }
}