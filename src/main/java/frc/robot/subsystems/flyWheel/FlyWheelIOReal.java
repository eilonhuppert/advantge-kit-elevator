package frc.robot.subsystems.flyWheel;
import static frc.robot.subsystems.flyWheel.FlyWheelConstants.*;
import static frc.robot.subsystems.flyWheel.FlyWheelConstants.FlyWheelRealConstants.*;


public class FlyWheelIOReal implements FlyWheelIO {

  public FlyWheelIOReal() {
     //todo tune
    // Apply the configuration to the motor
    motor.getConfigurator().apply(config);
  }
}
