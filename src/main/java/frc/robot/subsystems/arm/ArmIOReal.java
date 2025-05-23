package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.subsystems.arm.ArmConstants.ArmRealConstants.*;

public class ArmIOReal implements ArmIO {
  public ArmIOReal() {
    motor.getConfigurator().apply(config);
  }
}
