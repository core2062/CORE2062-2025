package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.HolderConstants;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

public class HolderSubsystem extends SubsystemBase{

    private TalonSRX pushBelt = new TalonSRX(HolderConstants.kBeltPort);

    public static AnalogInput photoEye = new AnalogInput(0);

    // Initialize the servo hub
    private ServoHub m_Hub = new ServoHub(2);
    
    // Obtain a servo channel controller
    private ServoChannel m_Channel1 = m_Hub.getServoChannel(ChannelId.kChannelId1);
    private ServoChannel m_Channel2 = m_Hub.getServoChannel(ChannelId.kChannelId2);

    public boolean gripperClosed = false;
    public boolean autoGripperClosed = false;
    public Timer closeDelay = new Timer();

    public HolderSubsystem(){
        //Allow power to Servos
        m_Channel2.setPowered(true);
        m_Channel1.setPowered(true);
        //Enabling the servos to allow movement
        m_Channel2.setEnabled(true);
        m_Channel1.setEnabled(true);
    }

    /**
     * Sets the position of the grippers between open and closed
     * @param servoAngle servo angle is between 1 and 2, 1 sets the gripper to be open and 2 to closed
     */
    public void setGripperPosition(int servoAngle){
        if (servoAngle == 1){
            m_Channel2.setPulseWidth(HolderConstants.kServoOpen.get(1500));
            m_Channel1.setPulseWidth(HolderConstants.kServoOpen.get(1500));
        } else if (servoAngle == 2){
            m_Channel2.setPulseWidth(HolderConstants.kServoClosed.get(1500));
            m_Channel1.setPulseWidth(HolderConstants.kServoClosed.get(1500));
        }
    }

    public boolean getPhotoeyeTriggered(){
        if (photoEye.getValue() > 2000){
            return false;
        } else if (photoEye.getValue() < 2000) {
            return true;
        } else {
            return false;
        }
    }
    
    @Override
    public void periodic() {
        if (m_Channel1.getPulseWidth() > 2000){
            gripperClosed = true;
        } else if (m_Channel1.getPulseWidth() < 1100){
            gripperClosed = false;
        }

        if (getPhotoeyeTriggered() && m_Channel1.getPulseWidth() < 2000 && autoGripperClosed == false){
            if (closeDelay.hasElapsed(0.6) || closeDelay.get() == 0){
                closeDelay.reset();
                closeDelay.start();
            }
            if (closeDelay.hasElapsed(0.5)) {
                setGripperPosition(2);
                autoGripperClosed = true;
            }
        } else if (!getPhotoeyeTriggered()){
            closeDelay.stop();
            closeDelay.reset();
            autoGripperClosed = false;
        }
    }

    public void runBelt(double speed){
        pushBelt.set(ControlMode.PercentOutput, speed);
    }
}
