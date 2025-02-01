package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorMovementCommand;
import frc.robot.constants.Constants.ElevatorConstants;;

public class ElevaterSubsystem extends SubsystemBase {
    private TalonSRX ElevatorMotorOne = new TalonSRX(0);
    private TalonSRX ElevatorMotorTwo = new TalonSRX(0);

    private Encoder elevatorEncoder1 = new Encoder(ElevatorConstants.kElevatorEncoder1[0], ElevatorConstants.kElevatorEncoder1[1]);
    private Encoder elevatorEncoder2 = new Encoder(ElevatorConstants.kElevatorEncoder2[0], ElevatorConstants.kElevatorEncoder2[1]);

    public static DoubleSupplier liftSpeed = () -> ElevatorConstants.kElevatorSpeed.get(0.0);

    public ElevaterSubsystem(){
        ElevatorMotorOne.configAllSettings(ElevatorConstants.motorConfigs.motorConfig);
        ElevatorMotorOne.setInverted(true);
        ElevatorMotorOne.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        ElevatorMotorOne.setNeutralMode(NeutralMode.Brake);
        
        ElevatorMotorTwo.configAllSettings(ElevatorConstants.motorConfigs.motorConfig);
        ElevatorMotorTwo.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        ElevatorMotorTwo.setNeutralMode(NeutralMode.Brake);
    }

    public void setLiftSpeed(double speed){
        if (ElevatorMotorOne.isFwdLimitSwitchClosed() == 1 && speed < 0){
            // ElevatorMotorOne.set(ControlMode.PercentOutput, 0);
            ElevatorMotorTwo.set(ControlMode.PercentOutput, 0);
        } else if (ElevatorMotorTwo.isRevLimitSwitchClosed() == 1 && speed > 0){
            ElevatorMotorOne.set(ControlMode.PercentOutput, 0);
            // ElevatorMotorTwo.set(ControlMode.PercentOutput, 0);
        } else {
            ElevatorMotorOne.set(ControlMode.PercentOutput, speed);
            ElevatorMotorTwo.set(ControlMode.PercentOutput, speed);
        }
    }

    public double getEncoderValue(){
        return elevatorEncoder1.getDistance();
    }

    public Command ElevatorPositionCommand(int desirdedPostion){
        Command moveElevator = new ElevatorMovementCommand(this, desirdedPostion);
        return moveElevator;
    }

    @Override
    public void periodic(){
    }
}
