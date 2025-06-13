// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  // singelton
  public static Arm instance;

  public static Arm getInstance(ArmIO io) {
    if (instance == null) {
      instance = new Arm(io);
    }
    return instance;
  }

  /** Creates a new Arm. */
  private Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInput(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), () -> io.stop());
  }

  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), () -> io.stop());
  }

  public Command setVelocity(double velocity) {
    return startEnd(() -> io.setVelocity(velocity), () -> io.stop());
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position));
  }
}
