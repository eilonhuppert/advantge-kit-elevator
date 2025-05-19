package frc.robot.subsystems.flyWheel;

import static frc.robot.subsystems.flyWheel.FlyWheelConstants.*;
import static frc.robot.subsystems.flyWheel.FlyWheelConstants.FlyWheelSimConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class FlyWheelIOSim implements FlyWheelIO {
  @SuppressWarnings("unchecked")
  private final FlywheelSim sim = new FlywheelSim(PLANT, GEAR_BOX);

  public FlyWheelIOSim() {
    // TODO: tune
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set brake mode to hold position when no power is applied
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Invert motor direction if necessary
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Set gear ratio: Sensor to mechanism and rotor to sensor
    config.Feedback.SensorToMechanismRatio = 5; // Example gear ratio
    config.Feedback.RotorToSensorRatio = 1.0; // Default for integrated sensor

    // Configure PID gains for velocity control
    config.Slot0.kP = 4; // Example value, tune as needed
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.12;
    config.Slot0.kV = 0.12; // Feedforward gain, tune as needed

    // Set Motion Magic parameters (if using Motion Magic)
    config.MotionMagic.MotionMagicCruiseVelocity = 20; // RPS
    config.MotionMagic.MotionMagicAcceleration = 40; // rot per sec³

    // Configure current limits
    // config.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    // Apply the configuration to the motor
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInput(FlyWheelIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    // update the system sim
    double motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    sim.setInputVoltage(motorVoltage);
    sim.update(0.02);

    // update the motor sim
    simMotor.setRotorVelocity(sim.getAngularVelocityRPM() / 60 * GEAR_RATIO);
    simMotor.setRotorAcceleration(sim.getAngularAcceleration().baseUnitMagnitude() * GEAR_RATIO);
    System.out.println(motor.getMotorVoltage().getValueAsDouble() + " " + motorVoltage);

    // update the battery
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    // update akit inputs
    inputs.MotorCurrent = motor.getTorqueCurrent().getValueAsDouble();
    inputs.MotorVoltge = motor.getMotorVoltage().getValueAsDouble();
    inputs.MotorVelocity = motor.getVelocity().getValueAsDouble();
    inputs.MotorAcceleration = motor.getAcceleration().getValueAsDouble();
  }
}
