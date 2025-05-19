package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSimConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500Foc(NUM_OF_MOTORS),
          GEAR_RATIO,
          MASS,
          WHEEL_RADIUS,
          MAXIMUM_HIGHT,
          MINIMUM_HIGHT,
          true,
          MINIMUM_HIGHT);

  // system control
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicExpoTorqueCurrentFOC positionRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  private final Follower followerRequest = new Follower(RIGHT_MOTOR_ID, REVERSED);

  private final boolean isBreakMode = true;

  public ElevatorIOSim() {
    sim.setState(0, 0);
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set brake mode
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Invert if necessary
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Feedback config (use integrated sensor)
    config.Feedback.SensorToMechanismRatio = 15; // gear ratio
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
    rightSimMotor.setSupplyVoltage(12);
    leftSimMotor.setSupplyVoltage(12);
  }

  @Override
  public void updateInput(ElevatorIOInputs inputs) {
    // Feed applied voltage from Talon into sim

    double motorVoltage = rightSimMotor.getMotorVoltage();

    sim.setInputVoltage(motorVoltage);
    sim.update(0.02);

    double motorRotations = sim.getPositionMeters() / (2 * Math.PI * WHEEL_RADIUS) * GEAR_RATIO;
    double motorRPM =
        sim.getVelocityMetersPerSecond() / (2 * Math.PI * WHEEL_RADIUS) * GEAR_RATIO * 60;
    rightSimMotor.setRawRotorPosition(motorRotations);
    rightSimMotor.setRotorVelocity(motorRPM / 60);
    System.out.println(rightMotor.getVelocity());
    System.out.println(rightMotor.getPosition());

    inputs.MotorPosotion = Units.rotationsToRadians(rightMotor.getPosition().getValueAsDouble());
    inputs.MotorVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(rightMotor.getVelocity().getValueAsDouble());
    inputs.MotorVoltge = motorVoltage;
    inputs.MotorCurrent = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    rightMotor.setVoltage(voltage);
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
