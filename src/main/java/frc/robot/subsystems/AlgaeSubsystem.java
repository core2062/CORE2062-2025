package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.COREConstants;
import frc.robot.constants.Constants.AlgaeConstants;

// import frc.robot.constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase{
    //var config = new TalonFXConfiguration();
    
    //Commented code is from old one-motor layout
    private TalonSRX leftAlgaeMotor = new TalonSRX(AlgaeConstants.kLeftAlgaeMotorPort);
    private TalonSRX rightAlgaeMotor = new TalonSRX(AlgaeConstants.kRightAlgaeMotorPort);
    private TalonFX pivotAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaePivotMotorPort);
    
    DutyCycleOut m_PivotRequest = new DutyCycleOut(0);

    public AlgaeSubsystem(){
        leftAlgaeMotor.getAllConfigs(new TalonSRXConfiguration());
        leftAlgaeMotor.setInverted(true);
        rightAlgaeMotor.getAllConfigs(new TalonSRXConfiguration());
        rightAlgaeMotor.setInverted(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotAlgaeMotor.getConfigurator().apply(config);
    }

    public void setAlgaeMotorSpeed(double speed){
        leftAlgaeMotor.set(ControlMode.PercentOutput, speed);
        rightAlgaeMotor.set(ControlMode.PercentOutput, speed);
    }    
    public void setAlgaePivotMotorSpeed(double speed){
        pivotAlgaeMotor.setControl(m_PivotRequest.withOutput(speed));
    } 
    public void setAlgaePivotMotorSpeed(COREConstants speed){
        pivotAlgaeMotor.setControl(m_PivotRequest.withOutput(speed.get(0.0)));
    } 

    public double getRotation(){
        return (pivotAlgaeMotor.getPosition().getValueAsDouble()) / 100;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getRotation());
        if(getRotation() < 0.06 || getRotation() > 0.35){
            AlgaeConstants.kHoldSpeed.set(0.0);
        } else if (getRotation() > 0.06 && getRotation() < 0.35){
            AlgaeConstants.kHoldSpeed.set(0.02);
        }
    }
}
