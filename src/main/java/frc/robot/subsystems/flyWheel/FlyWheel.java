// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flyWheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FlyWheel extends SubsystemBase {
  private final FlyWheelIO io;
  private final FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();

  // singelton
  private static FlyWheel instance;

  public static FlyWheel getInstance(FlyWheelIO io) {
    if (instance == null) {
      instance = new FlyWheel(io);
    }
    return instance;
  }

  /** Creates a new FlyWheel. */
  private FlyWheel(FlyWheelIO io) {
    this.io = io;

    // todo: add configs
  }

  @Override
  public void periodic() {
    io.updateInput(inputs);
    Logger.processInputs("FlyWheel", inputs);
  }

  /**
   * Set the voltage of the motor in volts.
   *
   * @param voltage the motor voltage in volts
   * @return a command that sets the motor voltage
   */
  public Command setVoltage(double voltage) {
    return startEnd(() -> io.setVoltage(voltage), () -> io.stop());
  }

  /**
   * Set the current of the motor in amperes.
   *
   * @param current the motor current in amperes
   * @return a command that sets the motor current
   */
  public Command setCurrent(double current) {
    return startEnd(() -> io.setCurrent(current), () -> io.stop());
  }

  /**
   * Set the speed of the motor in rotations/s.
   *
   * @param speed the motor speed in rotations/s
   * @return a command that sets the motor speed
   */
  public Command setSpeed(double speed) {
    return startEnd(() -> io.setSpeed(speed), () -> io.stop());
  }
}
