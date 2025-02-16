package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorMovementCommand;
import frc.robot.constants.Constants.ElevatorConstants;;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX LeftElevatorMotor = new TalonFX(ElevatorConstants.kLeftLiftMotorPort);
    private TalonFX RightElevatorMotor = new TalonFX(ElevatorConstants.kRightLiftMotorPort);

    private DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    private DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort + 1);

    private Encoder leftElevatorEncoder = new Encoder(ElevatorConstants.kLeftElevatorEncoder[0], ElevatorConstants.kLeftElevatorEncoder[1]);
    private Encoder rightElevatorEncoder = new Encoder(ElevatorConstants.kRightElevatorEncoder[0], ElevatorConstants.kRightElevatorEncoder[1]);

    public static DoubleSupplier liftSpeed = () -> ElevatorConstants.kElevatorSpeed.get(0.0);

    DutyCycleOut m_request = new DutyCycleOut(0);
    MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

    public ElevatorSubsystem(){
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        var slotConfigs = config.Slot0;

        slotConfigs.kS = 0.24;
        slotConfigs.kV = 0.12;
        slotConfigs.kP = 4.8;
        slotConfigs.kI = 0;
        slotConfigs.kD = 0.1;

        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        LeftElevatorMotor.getConfigurator().apply(config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        LeftElevatorMotor.getConfigurator().setPosition(0);
        RightElevatorMotor.getConfigurator().apply(config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
        RightElevatorMotor.getConfigurator().setPosition(0);
        configEncoders();
        System.out.println(config.toString());
    }
    
    public void setLiftSpeed(double speed){
        LeftElevatorMotor.setControl(m_request.withOutput(speed));
        RightElevatorMotor.setControl(m_request.withOutput(speed));
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
    
    public void moveToHeight(double desiredHeight){
        LeftElevatorMotor.setControl(m_motmag.withPosition(heightToRotations(desiredHeight)));
        RightElevatorMotor.setControl(m_motmag.withPosition(heightToRotations(desiredHeight)));
    }
    
    public double heightToRotations(double height){
        double rOutput = height/ElevatorConstants.kHeightOutput;
        double motorRotations = 10 * rOutput;
        return motorRotations;
    }
    
    @Override
    public void periodic() {
        m_motmag.Slot = 0;
        SmartDashboard.putNumber("Encoder Left Value:", getEncoderValue());
        SmartDashboard.putNumber("Encoder Right Value:", getEncoderValue1());
        SmartDashboard.putNumber("Encoder difference:", getEncoderValue() - getEncoderValue1());
        SmartDashboard.putNumber("Motor Position 1:", LeftElevatorMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Motor Position 2:", RightElevatorMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator running 1", LeftElevatorMotor.getMotionMagicIsRunning().getValueAsDouble());
        SmartDashboard.putNumber("elevator running 2", RightElevatorMotor.getMotionMagicIsRunning().getValueAsDouble());
    }
}