package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorMovementCommand;
import frc.robot.constants.Constants.ElevatorConstants;;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX LeftElevatorMotor = new TalonFX(ElevatorConstants.kLeftLiftMotorPort);
    private TalonFX RightElevatorMotor = new TalonFX(ElevatorConstants.kRightLiftMotorPort);

    public static DoubleSupplier liftSpeed = () -> ElevatorConstants.kElevatorSpeed.get(0.0);

    private String ReverseLimitSwitch = "";
    public boolean atDesiredPose;
    public double desiredPos;

    DutyCycleOut m_request = new DutyCycleOut(0);
    MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

        public ElevatorSubsystem(){
            m_motmag.Slot = 0;
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
            var slotConfigs = config.Slot0;
    
            slotConfigs.kG = 0.275;
            slotConfigs.kS = 0.22;

            slotConfigs.kV = 1.2225;
            slotConfigs.kA = 0.112712;

            slotConfigs.kP = 6.85056;
            slotConfigs.kI = 0.0;
            slotConfigs.kD = 0.0;

            config.Slot0 = slotConfigs;
    
            var motionMagicConfigs = config.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = 50;
            motionMagicConfigs.MotionMagicAcceleration = 100;
            motionMagicConfigs.MotionMagicJerk = 160;
    
            config.MotionMagic = motionMagicConfigs;
    
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            LeftElevatorMotor.getConfigurator().apply(config);
            LeftElevatorMotor.getConfigurator().setPosition(0);
    
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            RightElevatorMotor.getConfigurator().apply(config);
            RightElevatorMotor.getConfigurator().setPosition(0);

            LeftElevatorMotor.setControl(new StrictFollower(RightElevatorMotor.getDeviceID()));
        }
        
        public void setLiftSpeed(double speed){
            RightElevatorMotor.setControl(m_request.withOutput(speed));
        }
        
        public Command elevatorLift(int coralPos){
            Command elevatorMovementCommand = new ElevatorMovementCommand(this, coralPos);
            return elevatorMovementCommand;
        }
        
        public void moveToHeight(double desiredHeight){
            desiredPos = desiredHeight;
            RightElevatorMotor.setControl(m_motmag.withPosition(heightToRotations(desiredHeight)));
        }
    
        public void holdHeight(){
            RightElevatorMotor.setControl(m_motmag.withPosition(RightElevatorMotor.getPosition().getValueAsDouble()));
        }
        
        public double heightToRotations(double height){
            double heightOffset = height - 19.75;
            double rOutput = heightOffset/ElevatorConstants.kHeightOutput;
            double motorRotations = 10 * rOutput;
            return motorRotations;
        }

        public double rotationsToHeight(double pos){
            double npos = pos/10;
            double height = npos * ElevatorConstants.kHeightOutput;
            return height + 19.75;
        }
        
        @Override
        public void periodic() {
            SmartDashboard.putNumber("Motor Position 1:", rotationsToHeight(LeftElevatorMotor.getRotorPosition().getValueAsDouble()));
            SmartDashboard.putNumber("Motor Position 2:", rotationsToHeight(RightElevatorMotor.getRotorPosition().getValueAsDouble()));
            // SmartDashboard.putNumber("elevator running 1", LeftElevatorMotor.getMotionMagicIsRunning().getValueAsDouble());
            // SmartDashboard.putNumber("elevator running 2", RightElevatorMotor.getMotionMagicIsRunning().getValueAsDouble());
            if (LeftElevatorMotor.getReverseLimit().toString().contains("ClosedToGround") || RightElevatorMotor.getReverseLimit().toString().contains("ClosedToGround")){
                if (ReverseLimitSwitch.contains("Open")){
                    RightElevatorMotor.setControl(m_request.withOutput(0));
                }
            }
            SmartDashboard.putNumber("desired elevator height", desiredPos);
            atDesiredPose = (Math.abs(desiredPos - rotationsToHeight(RightElevatorMotor.getPosition().getValueAsDouble())) <= (desiredPos * 0.05)) ? true : false;
            SmartDashboard.putBoolean("Desired Pose within range", atDesiredPose);
            ReverseLimitSwitch = RightElevatorMotor.getReverseLimit().toString();
        }
}