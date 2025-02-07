package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmConstants;;

public class ArmSubsystem extends SubsystemBase{
    private TalonSRX armRotationMotor = new TalonSRX(ArmConstants.kArmMotorPort);
    private Encoder armEncoder = new Encoder(ArmConstants.kArmEncoder[0], ArmConstants.kArmEncoder[1]);
    public double getEncoderValue() {
        return armEncoder.getDistance();
    }
    public void setAngleSpeed(double speed) {
        armRotationMotor.set(ControlMode.PercentOutput, speed);
    }
}
