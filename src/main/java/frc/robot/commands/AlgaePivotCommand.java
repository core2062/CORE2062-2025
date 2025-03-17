package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaePivotCommand extends Command{
    private AlgaeSubsystem a_AlgaePivot;
    private double speed;
    public AlgaePivotCommand(AlgaeSubsystem a_AlgaePivot, double speed){
        this.a_AlgaePivot = a_AlgaePivot;
        addRequirements(a_AlgaePivot);
        this.speed = speed;
    }

    @Override
    public void execute() {
        a_AlgaePivot.setAlgaeMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        a_AlgaePivot.setAlgaeMotorSpeed(0);
    }
}
