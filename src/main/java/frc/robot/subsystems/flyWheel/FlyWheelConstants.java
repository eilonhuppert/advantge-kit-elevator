package frc.robot.subsystems.flyWheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants;

public class FlyWheelConstants {
  // motor settings
  public static final int MOTOR_ID = 10;
  public static final TalonFX motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);

  // more technical motor settings
  public static final double GEAR_RATIO = 5;

  // system control
  public static final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

  public class FlyWheelRealConstants {
    public static final TalonFXConfiguration config = new TalonFXConfiguration();
    static {

    // Set brake mode to hold position when no power is applied
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Invert motor direction if necessary
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Set gear ratio: Sensor to mechanism and rotor to sensor
    config.Feedback.SensorToMechanismRatio = 5; // Example gear ratio
    config.Feedback.RotorToSensorRatio = 1.0; // Default for integrated sensor

    // Configure PID gains for velocity control
    config.Slot0.kP = 4; // Example value, tune as needed
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.5;
    config.Slot0.kS = 0.4;
    config.Slot0.kV = 0.4; // Feedforward gain, tune as needed

    // Set Motion Magic parameters (if using Motion Magic)
    config.MotionMagic.MotionMagicCruiseVelocity = 20; // RPS
    config.MotionMagic.MotionMagicAcceleration = 40; // rot per sec³

    // Configure current limits
    // config.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    } 
  }

  public class FlyWheelSimConstants {
    public static final TalonFXSimState simMotor = motor.getSimState();

    // system settings
    public static final DCMotor GEAR_BOX = DCMotor.getFalcon500Foc(1);
    /*
     * The parameter JKgMetersSquared represents the moment of inertia of the flywheel, measured in kilogram meters squared (kg·m²).
     *  In physics and control systems, the moment of inertia (commonly denoted as J)
     *  quantifies how much torque is required to change the rotational velocity of an object — in this case,
     *  your flywheel.
     */
    public static final double WHEEL_RADIUS = 0.03; // meters
    public static final double SYSTEM_MASS = 3; // kg
    public static final double J = 0.5 * SYSTEM_MASS * WHEEL_RADIUS * WHEEL_RADIUS;

    @SuppressWarnings("rawtypes")
    public static final LinearSystem PLANT =
        LinearSystemId.createFlywheelSystem(GEAR_BOX, J, GEAR_RATIO);

    public static final TalonFXConfiguration config = new TalonFXConfiguration();
    static {

    // Set brake mode to hold position when no power is applied
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Invert motor direction if necessary
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Set gear ratio: Sensor to mechanism and rotor to sensor
    config.Feedback.SensorToMechanismRatio = 5; // Example gear ratio
    config.Feedback.RotorToSensorRatio = 1.0; // Default for integrated sensor

    // Configure PID gains for velocity control
    config.Slot0.kP = 4; // Example value, tune as needed
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.5;
    config.Slot0.kS = 0.4;
    config.Slot0.kV = 0.4; // Feedforward gain, tune as needed

    // Set Motion Magic parameters (if using Motion Magic)
    config.MotionMagic.MotionMagicCruiseVelocity = 20; // RPS
    config.MotionMagic.MotionMagicAcceleration = 40; // rot per sec³

    // Configure current limits
    // config.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    } 
  }
}
