package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ElevatorMovementCommand;
import frc.robot.constants.Constants.ElevatorConstants;;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX LeftElevatorMotor = new TalonFX(ElevatorConstants.kLeftLiftMotorPort);
    private TalonFX RightElevatorMotor = new TalonFX(ElevatorConstants.kRightLiftMotorPort);

    // private DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    // private DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort + 1);

    // private Encoder leftElevatorEncoder = new Encoder(ElevatorConstants.kLeftElevatorEncoder[0], ElevatorConstants.kLeftElevatorEncoder[1]);
    // private Encoder rightElevatorEncoder = new Encoder(ElevatorConstants.kRightElevatorEncoder[0], ElevatorConstants.kRightElevatorEncoder[1]);

    public static DoubleSupplier liftSpeed = () -> ElevatorConstants.kElevatorSpeed.get(0.0);

    private String ReverseLimitSwitch = "";

    DutyCycleOut m_request = new DutyCycleOut(0);
    MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

    VoltageOut m_Volt = new VoltageOut(0);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_distance = Rotations.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine =
          new SysIdRoutine(
              // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                  // Tell SysId how to plumb the driving voltage to the motors.
                  voltage -> {
                    LeftElevatorMotor.setControl(m_Volt.withOutput(voltage));
                    RightElevatorMotor.setControl(m_Volt.withOutput(voltage));
                  },
                  // Tell SysId how to record a frame of data for each motor on the mechanism being
                  // characterized.
                  log -> {
                    // Record a frame for the left motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-left")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                LeftElevatorMotor.getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(m_distance.mut_replace(LeftElevatorMotor.getPosition().getValueAsDouble(), Rotations))
                        .angularVelocity(
                            m_velocity.mut_replace(LeftElevatorMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                    // Record a frame for the right motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-right")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                RightElevatorMotor.getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(m_distance.mut_replace(RightElevatorMotor.getPosition().getValueAsDouble(), Rotations))
                        .angularVelocity(
                            m_velocity.mut_replace(RightElevatorMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                  },
                  // Tell SysId to make generated commands require this subsystem, suffix test state in
                  // WPILog with this subsystem's name ("drive")
                  this));
    
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
            // System.out.println(config.toString());
            // System.out.println(LeftElevatorMotor.getConfigurator().equals(config));
            // System.out.println(RightElevatorMotor.getConfigurator().equals(config));
            // configEncoders();
        }
        
        public void setLiftSpeed(double speed){
            // LeftElevatorMotor.setControl(m_request.withOutput(speed));
            RightElevatorMotor.setControl(m_request.withOutput(speed));
        }
        
        // public double getEncoderValue(){
        //     return rightElevatorEncoder.getDistance();
        // }
        
        // public double getEncoderValue1(){
        //     return leftElevatorEncoder.getDistance();
        // }
        
        // public void resetEncoder(){
        //     rightElevatorEncoder.reset();
        //     leftElevatorEncoder.reset();
        // }
        
        // public void configEncoders(){
        //     // Configures the encoder to return a distance of 4 for every 256 pulses
        //     // Also changes the units of getRate
        //     rightElevatorEncoder.setDistancePerPulse(1/256);
        //     // Configures the encoder to consider itself stopped after .1 seconds
        //     // elevatorEncoder1.(0.1);
        //     // Configures the encoder to consider itself stopped when its rate is below 10
        //     rightElevatorEncoder.setMinRate(10);
        //     // Reverses the direction of the encoder
        //     rightElevatorEncoder.setReverseDirection(true);
        //     // Configures an encoder to average its period measurement over 5 samples
        //     // Can be between 1 and 127 samples
        //     rightElevatorEncoder.setSamplesToAverage(5);
            
        //     // Configures the encoder to return a distance of 4 for every 256 pulses
        //     // Also changes the units of getRate
        //     leftElevatorEncoder.setDistancePerPulse(1/256);
        //     // Configures the encoder to consider itself stopped after .1 seconds
        //     // elevatorEncoder1.(0.1);
        //     // Configures the encoder to consider itself stopped when its rate is below 10
        //     leftElevatorEncoder.setMinRate(10);
        //     // Reverses the direction of the encoder
        //     leftElevatorEncoder.setReverseDirection(false);
        //     // Configures an encoder to average its period measurement over 5 samples
        //     // Can be between 1 and 127 samples
        //     leftElevatorEncoder.setSamplesToAverage(5);
        // }
        
        public Command elevatorLift(int coralPos){
            Command elevatorMovementCommand = new ElevatorMovementCommand(this, coralPos);
            return elevatorMovementCommand;
        }
        
        public void moveToHeight(double desiredHeight){
            // LeftElevatorMotor.setControl(m_motmag.withPosition(heightToRotations(desiredHeight-19.5)));
            RightElevatorMotor.setControl(m_motmag.withPosition(heightToRotations(desiredHeight-19.5)));
        }
    
        public void holdHeight(){
            // LeftElevatorMotor.setControl(m_motmag.withPosition(LeftElevatorMotor.getPosition().getValueAsDouble()));
            RightElevatorMotor.setControl(m_motmag.withPosition(RightElevatorMotor.getPosition().getValueAsDouble()));
        }
        
        public double heightToRotations(double height){
            double rOutput = height/ElevatorConstants.kHeightOutput;
            double motorRotations = 10 * rOutput;
            return motorRotations;
        }

        public double rotationsToHeight(double pos){
            double npos = pos/10;
            double height = npos * ElevatorConstants.kHeightOutput;
            return height + 19.5;
        }
        
        @Override
        public void periodic() {
            // SmartDashboard.putNumber("Encoder Left Value:", getEncoderValue());
            // SmartDashboard.putNumber("Encoder Right Value:", getEncoderValue1());
            // SmartDashboard.putNumber("Encoder difference:", getEncoderValue() - getEncoderValue1());
            SmartDashboard.putNumber("Motor Position 1:", rotationsToHeight(LeftElevatorMotor.getRotorPosition().getValueAsDouble()));
            SmartDashboard.putNumber("Motor Position 2:", rotationsToHeight(RightElevatorMotor.getRotorPosition().getValueAsDouble()));
            SmartDashboard.putNumber("elevator running 1", LeftElevatorMotor.getMotionMagicIsRunning().getValueAsDouble());
            SmartDashboard.putNumber("elevator running 2", RightElevatorMotor.getMotionMagicIsRunning().getValueAsDouble());
            if (LeftElevatorMotor.getReverseLimit().toString().contains("ClosedToGround") || RightElevatorMotor.getReverseLimit().toString().contains("ClosedToGround")){
                if (ReverseLimitSwitch.contains("Open")){
                    RightElevatorMotor.setControl(m_request.withOutput(0));
                }
            }
            ReverseLimitSwitch = RightElevatorMotor.getReverseLimit().toString();
        }
    
          /**
       * Returns a command that will execute a quasistatic test in the given direction.
       *
       * @param direction The direction (forward or reverse) to run the test in
       */
        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return m_sysIdRoutine.quasistatic(direction);
        }
    
        /**
         * Returns a command that will execute a dynamic test in the given direction.
         *
         * @param direction The direction (forward or reverse) to run the test in
         */
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return m_sysIdRoutine.dynamic(direction);
    }
}