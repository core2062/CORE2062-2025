package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorMovementCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConfigs;

public class ElevaterSubsystem extends SubsystemBase {
    private TalonFX ElevatorMotorOne = new TalonFX(0);
    private TalonFX ElevatorMotorTwo = new TalonFX(0);

    private ElevatorConfigs motor1 = new ElevatorConfigs(1);
    private ElevatorConfigs motor2 = new ElevatorConfigs(2);

    private DigitalInput limitSwitch = new DigitalInput(Constants.ElevatorConstants.kLimitSwitchPort);

    private Encoder elevatorEncoder1 = new Encoder(Constants.ElevatorConstants.kElevatorEncoder1[0], Constants.ElevatorConstants.kElevatorEncoder1[1]);
    private Encoder elevatorEncoder2 = new Encoder(Constants.ElevatorConstants.kElevatorEncoder2[0], Constants.ElevatorConstants.kElevatorEncoder2[1]);

    public static DoubleSupplier liftSpeed = () -> Constants.ElevatorConstants.kElevatorSpeed.get(0.0);

    DutyCycleOut m_request = new DutyCycleOut(0);

    public ElevaterSubsystem(){
        ElevatorMotorOne.getConfigurator().apply(motor1.motorConfig);
        ElevatorMotorOne.getConfigurator().setPosition(0);

        ElevatorMotorTwo.getConfigurator().apply(motor2.motorConfig);
        ElevatorMotorTwo.getConfigurator().setPosition(0);
    }

    public void setLiftSpeed(double speed){
        if (limitSwitch.get() && speed < 0){
            ElevatorMotorOne.setControl(m_request.withOutput(0));
            ElevatorMotorTwo.setControl(m_request.withOutput(0));
        } else {
            ElevatorMotorOne.setControl(m_request.withOutput(-speed));
            ElevatorMotorTwo.setControl(m_request.withOutput(speed));
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
