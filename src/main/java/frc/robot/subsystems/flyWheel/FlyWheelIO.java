package frc.robot.subsystems.flyWheel;

import static frc.robot.subsystems.flyWheel.FlyWheelConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface FlyWheelIO {
  @AutoLog
  public static class FlyWheelIOInputs {
    public double MotorVoltge; // volts
    public double MotorCurrent; // amps
    public double MotorVelocity; // rps
    public double MotorAcceleration; // rad/(s*s)
  }

  default void updateInput(FlyWheelIOInputs inputs) {
    inputs.MotorCurrent = motor.getTorqueCurrent().getValueAsDouble();
    inputs.MotorVoltge = motor.getSupplyCurrent().getValueAsDouble();
    inputs.MotorVelocity = motor.getVelocity().getValueAsDouble();
    inputs.MotorAcceleration = motor.getAcceleration().getValueAsDouble();
  }

  /**
   * Set the voltage of the motor in volts this works for simulation && for the real robot
   *
   * @param voltage the motor voltage in volts
   */
  default void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Set the current of the motor in amperes. this works for simulation && for the real robot
   *
   * @param current the motor current in amperes
   */
  default void setCurrent(double current) {
    motor.setControl(currentRequest.withOutput(current));
  }

  /**
   * Set the speed of the motor in rotations/s. this works for simulation && for the real robot
   *
   * @param speed the motor speed in rotations/s
   */
  default void setSpeed(double speed) {
    motor.setControl(velocityRequest.withVelocity(speed));
  }

  /** Stop the motor. */
  default void stop() {
    motor.stopMotor();
  }
}
