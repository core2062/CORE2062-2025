package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.Constants;

public class HolderSubsystem {
    private Servo gripper = new Servo(Constants.HolderConstants.kGripperPort);
    private TalonSRX pushBelt = new TalonSRX(Constants.HolderConstants.kBeltPort);
    public HolderSubsystem(){
    }

    public void setGripperPosition(double servoAngle){
        gripper.setAngle(servoAngle);
    }

    public void runBelt(double speed){
        pushBelt.set(ControlMode.PercentOutput, speed);
    }
}
