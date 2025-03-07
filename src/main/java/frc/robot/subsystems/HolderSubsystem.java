package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.Constants.HolderConstants;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

public class HolderSubsystem {
    // private Servo gripper = new Servo(HolderConstants.kGripperPort);
    private TalonSRX pushBelt = new TalonSRX(HolderConstants.kBeltPort);

    // Initialize the servo hub
    private ServoHub m_Hub = new ServoHub(2);
    
    // Obtain a servo channel controller
    private ServoChannel m_Channel0 = m_Hub.getServoChannel(ChannelId.kChannelId0);
    private ServoChannel m_Channel1 = m_Hub.getServoChannel(ChannelId.kChannelId1);

    public HolderSubsystem(){
        //Allow power to Servos
        m_Channel0.setPowered(true);
        m_Channel1.setPowered(true);
        //Enabling the servos to allow movement
        m_Channel0.setEnabled(true);
        m_Channel1.setEnabled(true);
    }

    public void setGripperPosition(int servoAngle){
        if (servoAngle == 1){
            m_Channel0.setPulseWidth(HolderConstants.kServoOpen.get(1500));
            m_Channel1.setPulseWidth(HolderConstants.kServoOpen.get(1500));
        } else if (servoAngle == 2){
            m_Channel0.setPulseWidth(HolderConstants.kServoClosed.get(1500));
            m_Channel1.setPulseWidth(HolderConstants.kServoClosed.get(1500));
        }
    }

    public void runBelt(double speed){
        pushBelt.set(ControlMode.PercentOutput, speed);
    }
}
