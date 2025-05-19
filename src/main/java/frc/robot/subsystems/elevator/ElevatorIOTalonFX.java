package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorRealConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

  // log values
  private final StatusSignal<Angle> rightPosition = rightMotor.getPosition();
  private final StatusSignal<AngularVelocity> rightVelocity = rightMotor.getVelocity();
  private final StatusSignal<Voltage> rightVoltage = rightMotor.getMotorVoltage();
  private final StatusSignal<Current> rightCurrent = rightMotor.getStatorCurrent();

  // system control
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicExpoTorqueCurrentFOC positionRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  private final Follower followerRequest = new Follower(RIGHT_MOTOR_ID, REVERSED);

  private boolean isBreakMode = true;

  public ElevatorIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set brake mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Invert if necessary
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Feedback config (use integrated sensor)
    config.Feedback.SensorToMechanismRatio = 1.0 / 15.0; // gear ratio
    config.Feedback.RotorToSensorRatio = 1.0; // default for Falcon integrated sensor

    // Motion Magic config (tune these values as needed)
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // m/s
    config.MotionMagic.MotionMagicAcceleration = 2.0; // m/s^2
    config.MotionMagic.MotionMagicJerk = 100.0; // m/s^3 (optional)

    // PID slot 0 for MotionMagic
    config.Slot0.kP = 100.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.0; // Optional feedforward

    // Current limiting (optional safety)
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightMotor.getConfigurator().apply(config);
    leftMotor.setControl(followerRequest);
  }

  @Override
  public void updateInput(ElevatorIOInputs inputs) {
    inputs.MotorPosotion = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.MotorVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(rightVelocity.getValueAsDouble());
    inputs.MotorVoltge = rightVoltage.getValueAsDouble();
    inputs.MotorCurrent = rightCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    rightMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setCurrent(double current) {
    rightMotor.setControl(currentRequest.withOutput(current));
  }

  @Override
  public void setSpeed(double speed) {
    rightMotor.setControl(velocityRequest.withVelocity(speed));
  }

  @Override
  public void setPostion(double position) {
    rightMotor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void breakMode() {
    if (isBreakMode) {
      leftMotor.setNeutralMode(NeutralModeValue.Coast);
      rightMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      leftMotor.setNeutralMode(NeutralModeValue.Brake);
      rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void stop() {
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }
}
