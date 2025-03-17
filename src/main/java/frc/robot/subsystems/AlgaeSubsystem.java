package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.AlgaeConstants;

// import frc.robot.constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase{
    //var config = new TalonFXConfiguration();
    
    //Commented code is from old one-motor layout
    private TalonFX leftAlgaeMotor = new TalonFX(AlgaeConstants.kLeftAlgaeMotorPort);
    private TalonFX rightAlgaeMotor = new TalonFX(AlgaeConstants.kRightAlgaeMotorPort);
    //private TalonFX intakeAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaeIntakeMotorPort);
    private TalonFX pivotAlgaeMotor = new TalonFX(AlgaeConstants.kAlgaePivotMotorPort);
    

    DutyCycleOut m_IntakeRequest = new DutyCycleOut(0);
    DutyCycleOut m_PivotRequest = new DutyCycleOut(0);

    public AlgaeSubsystem(){
        // var config = ;

        leftAlgaeMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        rightAlgaeMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
        pivotAlgaeMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        //intakeAlgaeMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    }

    public void setAlgaeMotorSpeed(double speed){
        //intakeAlgaeMotor.setControl(m_request.withOutput(speed));
        leftAlgaeMotor.setControl(m_IntakeRequest.withOutput(speed));
        rightAlgaeMotor.setControl(m_IntakeRequest.withOutput(speed));
    }    
    public void setAlgaePivotMotorSpeed(double speed){
        pivotAlgaeMotor.setControl(m_PivotRequest.withOutput(speed));
    } 
}
