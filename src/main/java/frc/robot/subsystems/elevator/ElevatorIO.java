package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    // right motor
    public double MotorPosotion; // (rotations)
    public double MotorVoltge; // (volts)
    public double MotorCurrent; // (amps)
    public double MotorVelocity; // (rpm)
  }

  public default void updateInput(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setVoltage(DoubleSupplier voltage) {}

  public default void setCurrent(double current) {}

  public default void setCurrent(DoubleSupplier current) {}

  public default void setSpeed(double speed) {}

  public default void setSpeed(DoubleSupplier speed) {}

  public default void setPostion(double position) {}

  public default void setPostion(DoubleSupplier position) {}

  public default void breakMode() {}

  public default void stop() {}
}
