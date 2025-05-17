package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSimConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
          0);

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
    leftMotor.setControl(followerRequest);
    // TODO: configs
  }

  @Override
  public void updateInput(ElevatorIOInputs inputs) {
    // Feed applied voltage from Talon into sim
    double motorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    sim.setInputVoltage(motorVoltage);
    sim.update(0.02);

    double motorRotations = sim.getPositionMeters() / (2 * Math.PI * WHEEL_RADIUS) * GEAR_RATIO;
    double motorRPM =
        sim.getVelocityMetersPerSecond() / (2 * Math.PI * WHEEL_RADIUS) * GEAR_RATIO * 60;
    rightSimMotor.setRawRotorPosition(motorRotations);
    rightSimMotor.setRotorVelocity(motorRPM);

    inputs.MotorPosotion = Units.rotationsToRadians(motorRotations);
    inputs.MotorVelocity = Units.rotationsPerMinuteToRadiansPerSecond(motorRPM);
    inputs.MotorVoltge = motorVoltage;
    inputs.MotorCurrent = sim.getCurrentDrawAmps();
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
