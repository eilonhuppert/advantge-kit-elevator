package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.subsystems.arm.ArmConstants.ArmSimConstants.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmIOSim implements ArmIO {
  private double pastVelocity = 0;
  private MechanismLigament2d arm;

  public ArmIOSim() {
    motor.getConfigurator().apply(config);
    CreateMech2D();
  }

  void CreateMech2D() {
    Mechanism2d mech = new Mechanism2d(3, 3);

    MechanismRoot2d root = mech.getRoot("motor", 1.5, 0);

    MechanismLigament2d line = root.append(new MechanismLigament2d("line", 1.5, 90));
    arm =
        line.append(new MechanismLigament2d("arm", LEANGTH, 360, 6, new Color8Bit(Color.kPurple)));

    SmartDashboard.putData("Mech2d", mech);
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

    // updates Mechanism2d
    arm.setAngle(inputs.ArmPosition * 360 - 90);
  }
}
