package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.Constants.HolderConstants;;

public class HolderSubsystem {
    private Servo gripper = new Servo(HolderConstants.kGripperPort);
    private TalonSRX pushBelt = new TalonSRX(HolderConstants.kBeltPort);
    public HolderSubsystem(){
    }

    public void setGripperPosition(double servoAngle){
        if (servoAngle == 1){
            gripper.setAngle(HolderConstants.kServoOpen.get(0));
        } else if (servoAngle == 2){
            gripper.setAngle(HolderConstants.kServoClosed.get(0));
        }
    }

    public void runBelt(double speed){
        pushBelt.set(ControlMode.PercentOutput, speed);
    }
}
