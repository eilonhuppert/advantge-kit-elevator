package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double ArmPosition; // in rotations
    public double ArmVelocity; // in rotations/s
    public double ArmAcceleration; // in rotations/s^2
    public double ArmCurrent; // in amperes
    public double ArmVoltage; // in volts
  }

  default void updateInput(ArmIOInputs inputs) {
    inputs.ArmPosition = motor.getPosition().getValueAsDouble();
    inputs.ArmVelocity = motor.getVelocity().getValueAsDouble();
    inputs.ArmAcceleration = motor.getAcceleration().getValueAsDouble();
    inputs.ArmCurrent = motor.getTorqueCurrent().getValueAsDouble();
    inputs.ArmVoltage = motor.getMotorVoltage().getValueAsDouble();
  }

  default void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  default void setVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  default void setCurrent(double current) {
    motor.setControl(currentRequest.withOutput(current));
  }

  default void setPosition(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  default void stop() {
    motor.stopMotor();
  }
}
