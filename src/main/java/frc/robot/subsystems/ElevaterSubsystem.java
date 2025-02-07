package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ElevaterSubsystem extends SubsystemBase {
    private TalonFX ElevatorMotorOne = new TalonFX(0);
    private TalonFX ElevatorMotorTwo = new TalonFX(0);

    private DigitalInput upperLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kLimitSwitchPort);
    private DigitalInput lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.kLimitSwitchPort);

    private Encoder elevatorEncoder1 = new Encoder(Constants.ElevatorConstants.kElevatorEncoder1[0], Constants.ElevatorConstants.kElevatorEncoder1[1]);
    private Encoder elevatorEncoder2 = new Encoder(Constants.ElevatorConstants.kElevatorEncoder2[0], Constants.ElevatorConstants.kElevatorEncoder2[1]);

    public static DoubleSupplier liftSpeed = () -> Constants.ElevatorConstants.kElevatorSpeed.get(0.0);

    DutyCycleOut m_request = new DutyCycleOut(0);

    public ElevaterSubsystem(){
        ElevatorMotorOne.getConfigurator().apply(new TalonFXConfiguration());
        ElevatorMotorOne.getConfigurator().setPosition(0);
        ElevatorMotorTwo.getConfigurator().apply(new TalonFXConfiguration());
        ElevatorMotorTwo.getConfigurator().setPosition(0);
    }

    public void setLiftSpeed(double speed){
        ElevatorMotorOne.setControl(m_request.withOutput(speed)
                .withLimitForwardMotion(upperLimitSwitch.get())
                .withLimitReverseMotion(lowerLimitSwitch.get()));
        ElevatorMotorTwo.setControl(m_request.withOutput(speed)
                .withLimitForwardMotion(upperLimitSwitch.get())
                .withLimitReverseMotion(lowerLimitSwitch.get()));
    }

    public double getEncoderValue(){
        return elevatorEncoder1.getDistance();
    }
}