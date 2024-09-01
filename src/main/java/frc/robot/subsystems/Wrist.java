// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

    private TalonFX m_wristMotor;
  /** Creates a new Wrist. */
  public Wrist() {
    m_wristMotor = new TalonFX(WristConstants.wristTalonID);
    m_wristMotor.getConfigurator().apply(WristConstants.kWristConfiguration, 1);

    this.m_wristMotor.setPosition(0);
    this.m_wristMotor.set(0);
    //hi
  }
   /**
   * PID wrist to position
   * 
   * @param rotations 0 to 1 rotations
   */
  private void setAngle(double position) {
   m_wristMotor.setControl(WristConstants.wristPositionControl.withPosition(position));
  }

  /**
   * PID wrist to position
   * 
   * @param rotations Rotation 2d
   */
  public void setAngle(Rotation2d position) {
    setAngle(position.getRotations());
  }

  public void stow() {
    setAngle(0);
  }

  /**
   * Just PID to the current angle to hold position
   */
  public void holdPosition() {
    setAngle(getAngle());
  }

  public Rotation2d getSetpointError() {
    return Rotation2d.fromRotations(m_wristMotor.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError().getDegrees()) <= WristConstants.angleErrorTolerance.getDegrees();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_wristMotor.getPosition().getValue());
  }

  public double getVoltageOut() {
    return m_wristMotor.getMotorVoltage().getValue();
  }

  public void stop() {
    m_wristMotor.setControl(new DutyCycleOut(0));
  }

  public void setwristVoltage(double volts) {
    m_wristMotor.setControl(new VoltageOut(volts));
  }

  
  // To position for Intake, move Arm to INTAKE position
    public Command prepareForIntakeCommand() {
        return new RunCommand(()-> this.setAngle(Rotation2d.fromDegrees(WristConstants.WristIntakeAngle)), this)
            .until(()->this.isAtSetpoint());
    }   

    public Command stowwristCommand() {
      return new RunCommand(()->this.stow(), this);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("wristError", getSetpointError().getDegrees());
    SmartDashboard.putNumber("wristCurrentAngle", getAngle().getDegrees());
  }
}
