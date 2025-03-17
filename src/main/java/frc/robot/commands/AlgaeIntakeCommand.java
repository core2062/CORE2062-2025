package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeCommand extends Command{
    private AlgaeSubsystem a_AlgaeIntake;
    private double speed;
    public AlgaeIntakeCommand(AlgaeSubsystem a_AlgaeIntake, double speed){
        this.a_AlgaeIntake = a_AlgaeIntake;
        addRequirements(a_AlgaeIntake);
        this.speed = speed;
    }

    @Override
    public void execute() {
        a_AlgaeIntake.setAlgaeMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        a_AlgaeIntake.setAlgaeMotorSpeed(0);
    }
}
