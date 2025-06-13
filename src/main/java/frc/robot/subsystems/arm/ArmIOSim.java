package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.subsystems.arm.ArmConstants.ArmSimConstants.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ArmIOSim implements ArmIO {
  private double pastVelocity = 0;

  public ArmIOSim() {
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInput(ArmIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simMotor.getMotorVoltage());
    sim.update(0.02);

    // Calculate rotor position and velocity in rotations and RPS
    double rotorPosition = (sim.getAngleRads() * GEAR_RATIO) / (2 * Math.PI);
    double rotorVelocity = (sim.getVelocityRadPerSec() * GEAR_RATIO) / (2 * Math.PI);

    // Set simulated motor values
    simMotor.setRawRotorPosition(rotorPosition);
    simMotor.setRotorVelocity(rotorVelocity);

    // Compute acceleration
    double acceleration = (rotorVelocity - pastVelocity) / 0.02;
    pastVelocity = rotorVelocity;

    // Update battery voltage
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    // Populate inputs from the motor (which now reflects the sim)
    inputs.ArmPosition = motor.getPosition().getValueAsDouble();
    inputs.ArmVelocity = motor.getVelocity().getValueAsDouble();
    inputs.ArmAcceleration = acceleration;
    inputs.ArmCurrent = motor.getTorqueCurrent().getValueAsDouble();
    inputs.ArmVoltage = motor.getMotorVoltage().getValueAsDouble();
  }
}
