package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.Constants;

public class ArmSubsystem {
    private TalonSRX armRotationMotor = new TalonSRX(Constants.ArmConstants.kArmMotorPort);
    
}
