package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {


    public static final class ArmConstants {
        public static final int armLeaderID = 14; //right
        public static final int armFollowerID = 21; //left
        public static final String armTalonCANBus = "CAN0";
        public static final double armGearRatio = 25.64; // Sensor to Mechanism Ratio

        public static final double ArmIntakeAngle = 10;

        public static final double armMinClamp = -5;
        public static final double armMaxClamp = 123.05;

        public static final Rotation2d armMinAngle = Rotation2d.fromDegrees(armMinClamp);
        public static final Rotation2d armMaxAngle = Rotation2d.fromDegrees(armMaxClamp);

        public static final TalonFXConfiguration kArmConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0.22)
        .withMotionMagicAcceleration(0.22)
        .withMotionMagicJerk(0))
      .withSlot0(new Slot0Configs()
        .withKV(0)
        .withKA(0)
        .withKP(500) //1000, Brayden- Try 400 to stop tweety bird (aka over correcting) //Second thing to change after boolean and driver invert
        .withKI(0)
        .withKD(1) //Brayden- Try 50 to stop tweety bird (aka over correcting) //Second thing to change after boolean and driver invert
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(6)
        .withKS(0))
      .withFeedback(new FeedbackConfigs()
      .withSensorToMechanismRatio(armGearRatio))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(armMaxAngle.getRotations())
        .withReverseSoftLimitThreshold(armMinAngle.getRotations()));

        public static final MotionMagicExpoVoltage armPositionControl = new MotionMagicExpoVoltage(0);
        public static final Follower followerControl = new Follower(armLeaderID, true);
        public static final Rotation2d angleErrorTolerance = Rotation2d.fromDegrees(2.5); // Degrees
    }

    public static final class ShooterConstantsLeader {

    public static final int shooterTalonLeaderID = 17; //right
    public static final String shooterTalonCANBus = "rio";

    //public static final double shooterGearRatio = 0.5; // Sensor to Mechanism Ratio
    public static final double shooterGearRatio = 1;

    public static final double shooterVelocityTolerance = 0; // RPM

    public static final TalonFXConfiguration kShooterConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive))
        //.withInverted(InvertedValue.Clockwise_Positive)) ---Shyloh
      .withSlot0(new Slot0Configs()
        //.withKS(1) 4499: s=1, v=.2, p=12, 
        .withKV(.075) // 0.075
        .withKP(.205) // 0.125
        .withKI(0)
        .withKD(0))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(shooterGearRatio));

    public static final VelocityVoltage shooterControl = new VelocityVoltage(1000);
    public static final double kShooterVelocityUpdateFrequency = 10; // Hertz
  }

  public static final class ShooterConstantsFollower {

    public static final int shooterTalonFollowerID = 20; //left
    public static final String shooterTalonCANBus = "rio";

    //public static final double shooterGearRatio = 0.5; // Sensor to Mechanism Ratio
    public static final double shooterGearRatio = 1;

    public static final double shooterVelocityTolerance = 0; // RPM

    public static final TalonFXConfiguration kShooterConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
        //.withInverted(InvertedValue.Clockwise_Positive)) ---Shyloh
      .withSlot0(new Slot0Configs()
        //.withKS(1) 4499: s=1, v=.2, p=12, 
        .withKV(.075) // 0.075
        .withKP(.205) // 0.125
        .withKI(0)
        .withKD(0))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(shooterGearRatio));

    public static final VelocityVoltage shooterControl = new VelocityVoltage(5008);

    public static final double kShooterVelocityUpdateFrequency = 10; // Hertz
  }

  public static final class IntakeConstants {

    public static final int intakeTalonID = 15;
    public static final String intakeTalonCANBus = "rio";

    public static final int intakeSensorID = 1;
    //public static final RangingMode intakeSensorRange = RangingMode.Short;
    public static final double intakeSampleTime = 0;

    //public static final double isNotePresentTOF = 350; // Milimeters

    public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive));


    public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0.5);
    public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0.5);
  }
    public static final double kConfigTimeoutSeconds = 0.1;

	
}
