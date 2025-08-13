package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class ElevatorJoystickCommand extends Command{
    private final ElevatorSubsystem e_Elevator;
    private final DoubleSupplier m_speedSupplier;

    public ElevatorJoystickCommand(ElevatorSubsystem elevator, DoubleSupplier setLiftSpeed){
        e_Elevator = elevator;
        m_speedSupplier=setLiftSpeed;
        addRequirements(e_Elevator);
    }

    @Override
    public void execute() {
     e_Elevator.setLiftSpeed(m_speedSupplier.getAsDouble()*Constants.ElevatorConstants.kElevatorSpeed.get(0.0));
    }

    @Override
    public void end(boolean interrupted) {
     e_Elevator.setLiftSpeed(0.0);
    }

    @Override
    public boolean isFinished(){
     return false;
    }
}
