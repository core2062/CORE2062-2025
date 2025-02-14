package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorMovementCommand;
import frc.robot.constants.Constants.ElevatorConstants;;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX ElevatorMotorOne = new TalonFX(ElevatorConstants.kLeftLiftMotorPort);
    private TalonFX ElevatorMotorTwo = new TalonFX(ElevatorConstants.kRightLiftMotorPort);

    private DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    private DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort + 1);

    private Encoder leftElevatorEncoder = new Encoder(ElevatorConstants.kLeftElevatorEncoder[0], ElevatorConstants.kLeftElevatorEncoder[1]);
    private Encoder rightElevatorEncoder = new Encoder(ElevatorConstants.kRightElevatorEncoder[0], ElevatorConstants.kRightElevatorEncoder[1]);

    public static DoubleSupplier liftSpeed = () -> ElevatorConstants.kElevatorSpeed.get(0.0);

    DutyCycleOut m_request = new DutyCycleOut(0);

    public ElevatorSubsystem(){
        ElevatorMotorOne.getConfigurator().apply(new TalonFXConfiguration());
        ElevatorMotorOne.getConfigurator().setPosition(0);
        ElevatorMotorTwo.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        ElevatorMotorTwo.getConfigurator().setPosition(0);
        configEncoders();
    }

    public void setLiftSpeed(double speed){
        ElevatorMotorOne.setControl(m_request.withOutput(speed));
        ElevatorMotorTwo.setControl(m_request.withOutput(speed));
    }

    public double getEncoderValue(){
        return rightElevatorEncoder.getDistance();
    }

    public double getEncoderValue1(){
        return leftElevatorEncoder.getDistance();
    }

    public void resetEncoder(){
        rightElevatorEncoder.reset();
        leftElevatorEncoder.reset();
    }

    public void configEncoders(){
        // Configures the encoder to return a distance of 4 for every 256 pulses
        // Also changes the units of getRate
        rightElevatorEncoder.setDistancePerPulse(1/256);
        // Configures the encoder to consider itself stopped after .1 seconds
        // elevatorEncoder1.(0.1);
        // Configures the encoder to consider itself stopped when its rate is below 10
        rightElevatorEncoder.setMinRate(10);
        // Reverses the direction of the encoder
        rightElevatorEncoder.setReverseDirection(true);
        // Configures an encoder to average its period measurement over 5 samples
        // Can be between 1 and 127 samples
        rightElevatorEncoder.setSamplesToAverage(5);

        // Configures the encoder to return a distance of 4 for every 256 pulses
        // Also changes the units of getRate
        leftElevatorEncoder.setDistancePerPulse(1/256);
        // Configures the encoder to consider itself stopped after .1 seconds
        // elevatorEncoder1.(0.1);
        // Configures the encoder to consider itself stopped when its rate is below 10
        leftElevatorEncoder.setMinRate(10);
        // Reverses the direction of the encoder
        leftElevatorEncoder.setReverseDirection(false);
        // Configures an encoder to average its period measurement over 5 samples
        // Can be between 1 and 127 samples
        leftElevatorEncoder.setSamplesToAverage(5);
    }

    public Command elevatorLift(int coralPos){
        Command elevatorMovementCommand = new ElevatorMovementCommand(this, coralPos);
        return elevatorMovementCommand;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Left Value:", getEncoderValue());
        SmartDashboard.putNumber("Encoder Right Value:", getEncoderValue1());
        SmartDashboard.putNumber("Encoder difference:", getEncoderValue() - getEncoderValue1());
    }
}