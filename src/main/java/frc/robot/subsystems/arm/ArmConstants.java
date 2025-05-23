package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConstants {
    public static final int MOTOR_ID = 1;
    public static final TalonFX motor = new TalonFX(MOTOR_ID, "canBus");

    // system control
    public static final MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    public static final MotionMagicVelocityTorqueCurrentFOC velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(
            0);
    public static final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

    public class ArmRealConstants {
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final double GEAR_RATIO = 5;

        static {

            // Set brake mode to hold position when no power is applied
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            // Invert motor direction if necessary
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            // Set gear ratio: Sensor to mechanism and rotor to sensor
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO; // Example gear ratio
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

    public class ArmSimConstants {
        public static final TalonFXSimState simMotor = motor.getSimState();

        public static final double GEAR_RATIO = 5;
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final double JKG = 0; // todo: calculate with CAD
        public static final double LEANGTH = 0; // todo: find with CAD
        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 0;
        public static final double STARTING_ANGLE = 0;

        static { // todo: add soft limits

            // Set brake mode to hold position when no power is applied
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            // Invert motor direction if necessary
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            // Set gear ratio: Sensor to mechanism and rotor to sensor
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO; // Example gear ratio
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

        public static final SingleJointedArmSim sim = new SingleJointedArmSim(
                DCMotor.getFalcon500(1),
                GEAR_RATIO,
                JKG,
                LEANGTH,
                MIN_ANGLE,
                MAX_ANGLE,
                true,
                STARTING_ANGLE);

    }
}
