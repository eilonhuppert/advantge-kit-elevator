package frc.robot.subsystems.flyWheel;

import static frc.robot.subsystems.flyWheel.FlyWheelConstants.*;
import static frc.robot.subsystems.flyWheel.FlyWheelConstants.FlyWheelSimConstants.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class FlyWheelIOSim implements FlyWheelIO {
  @SuppressWarnings("unchecked")
  private final FlywheelSim sim = new FlywheelSim(PLANT, GEAR_BOX);

  public FlyWheelIOSim() {
    //todo tune
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
