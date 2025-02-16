package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.commands.ArmMovementCommand;

public class ArmSubsystem extends SubsystemBase{
    private TalonSRX armRotationMotor = new TalonSRX(ArmConstants.kArmMotorPort);
    private Encoder armEncoder = new Encoder(ArmConstants.kArmEncoder[0], ArmConstants.kArmEncoder[1]);

    // private TalonSRXConfiguration

    public ArmSubsystem(){
        armRotationMotor.setNeutralMode(NeutralMode.Brake);
        armRotationMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        armRotationMotor.setSelectedSensorPosition(0.0);
    }    

    public double getEncoderValue() {
        return armRotationMotor.getSelectedSensorPosition(0) * 0.087890625;
    }
    
    public void setAngleSpeed(double speed) {
        armRotationMotor.set(ControlMode.PercentOutput, speed);
    }

    public Command rotateArm(int armPos){
        Command rotate = new ArmMovementCommand(this, armPos);
        return rotate;
    }

    public void readf(double desiredAngle){
        armRotationMotor.set(ControlMode.MotionMagic, desiredAngle);
    }
}
