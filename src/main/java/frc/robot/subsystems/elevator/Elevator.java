// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public static Elevator instance;

  public static Elevator getInstance(ElevatorIO io) {
    if (instance == null) {
      instance = new Elevator(io);
    }
    return instance;
  }

  private Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInput(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), () -> io.stop());
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return runEnd(() -> io.setVoltage(voltage.getAsDouble()), () -> io.stop());
  }

  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), () -> io.stop());
  }

  public Command setCurrent(DoubleSupplier current) {
    return runEnd(() -> io.setCurrent(current.getAsDouble()), () -> io.stop());
  }

  public Command setSpeed(double speed) {
    return startEnd(() -> io.setSpeed(speed), () -> io.stop());
  }

  public Command setSpeed(DoubleSupplier speed) {
    return runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.stop());
  }

  public Command setPostion(double position) {
    return startEnd(() -> io.setPostion(position), () -> io.stop());
  }

  public Command setPostion(DoubleSupplier position) {
    return runEnd(() -> io.setPostion(position.getAsDouble()), () -> io.stop());
  }

  public Command breakMode() {
    return runOnce(() -> io.breakMode());
  }
}
