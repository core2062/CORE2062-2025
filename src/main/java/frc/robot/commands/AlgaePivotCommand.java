package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaePivotCommand extends Command {
    private final AlgaeSubsystem e_AlgaeSubsystem;
    private final DoubleSupplier m_algaeArmSupplier;
    private boolean a_nochange = true;

    public AlgaePivotCommand(AlgaeSubsystem algae, DoubleSupplier setArmSpeed){

        e_AlgaeSubsystem = algae;
        m_algaeArmSupplier = setArmSpeed;
        addRequirements(e_AlgaeSubsystem);
    }

    @Override
    public void execute() {
        System.out.println("In algae execute");
       double y = -m_algaeArmSupplier.getAsDouble()*Constants.AlgaeConstants.kAlgaeArmSpeed.get(0.6);
       if (Math.abs(y) > 0.2) {
        e_AlgaeSubsystem.setAlgaePivotMotorSpeed(y);
        a_nochange = false;
       } else if (!a_nochange) {
        e_AlgaeSubsystem.setAlgaePivotMotorSpeed(Constants.AlgaeConstants.kHoldSpeed);
        a_nochange = true;
       }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Holding algae");
        e_AlgaeSubsystem.setAlgaePivotMotorSpeed(Constants.AlgaeConstants.kHoldSpeed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
