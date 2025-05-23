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
        // update the system sim
        simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
        sim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
        sim.update(0.02);

        // update the motor sim
        pastVelocity = motor.getVelocity().getValueAsDouble();
        simMotor.setRotorVelocity((sim.getVelocityRadPerSec() * GEAR_RATIO) / 2 / Math.PI); // math might be worng
        simMotor.setRotorAcceleration((motor.getVelocity().getValueAsDouble() - pastVelocity) / 0.02);

        // update the battery
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        // update the output values
        inputs.ArmPosition = motor.getPosition().getValueAsDouble();
        inputs.ArmVelocity = motor.getVelocity().getValueAsDouble();
        inputs.ArmAcceleration = motor.getAcceleration().getValueAsDouble();
        inputs.ArmCurrent = motor.getTorqueCurrent().getValueAsDouble();
        inputs.ArmVoltage = motor.getMotorVoltage().getValueAsDouble();
    }
}