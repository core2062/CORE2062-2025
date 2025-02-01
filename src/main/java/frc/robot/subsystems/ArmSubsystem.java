package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmConstants;;

public class ArmSubsystem {
    private TalonSRX armRotationMotor = new TalonSRX(ArmConstants.kArmMotorPort);
    private Encoder armEncoder = new Encoder(ArmConstants.kArmEncoder[0], ArmConstants.kArmEncoder[1]);
}
