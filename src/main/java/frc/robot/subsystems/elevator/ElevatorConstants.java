package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class ElevatorConstants {
  public static final int RIGHT_MOTOR_ID = 0;
  public static final int LEFT_MOTOR_ID = 1;

  // motors
  public static final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID);
  public static final TalonFX leftMotor = new TalonFX(LEFT_MOTOR_ID);

  public class ElevatorRealConstants {
    public static final boolean REVERSED = true;
  }

  public class ElevatorSimConstants {
    public static final boolean REVERSED = false;
    public static final TalonFXSimState rightSimMotor = rightMotor.getSimState();
    public static final TalonFXSimState leftSimMotor = leftMotor.getSimState();

    public static final int NUM_OF_MOTORS = 2;
    public static final double GEAR_RATIO = 15;
    public static final double MASS = 15; // kg
    public static final double WHEEL_RADIUS = 0.015; // meters
    public static final double MAXIMUM_HIGHT = 10; // meters
    public static final double MINIMUM_HIGHT = 0;
  }
}
