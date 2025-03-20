package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HolderSubsystem;

public class ReleaseGripperFeedCommand extends Command{
    private HolderSubsystem h_holderSubsystem;
    private double speed;
    private Timer timer = new Timer();

    public ReleaseGripperFeedCommand(HolderSubsystem s_Subsystem, double speed){
        this.h_holderSubsystem = s_Subsystem;
        addRequirements(h_holderSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(h_holderSubsystem.gripperClosed == true){
            h_holderSubsystem.setGripperPosition(1);
        }
        if(h_holderSubsystem.gripperClosed == false && timer.get() > 0.5){
            h_holderSubsystem.runBelt(speed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        h_holderSubsystem.runBelt(0);
    }
}
