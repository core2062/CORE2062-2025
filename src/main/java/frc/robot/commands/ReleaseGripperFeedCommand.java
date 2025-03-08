package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HolderSubsystem;

public class ReleaseGripperFeedCommand extends Command{
    private HolderSubsystem h_holderSubsystem;
    private double speed;

    public ReleaseGripperFeedCommand(HolderSubsystem s_Subsystem, double speed){
        this.h_holderSubsystem = s_Subsystem;
        addRequirements(h_holderSubsystem);
        this.speed = speed;
    }

    @Override
    public void execute() {
        if(h_holderSubsystem.gripperClosed == true){
            h_holderSubsystem.setGripperPosition(1);
        }
        if(h_holderSubsystem.gripperClosed == false){
            h_holderSubsystem.runBelt(speed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        h_holderSubsystem.runBelt(0);
    }
}
