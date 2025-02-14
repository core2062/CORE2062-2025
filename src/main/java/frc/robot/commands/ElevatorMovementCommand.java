package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorMovementCommand extends Command{
    private ElevatorSubsystem l_Lift;
    private double DifferenceOfAngle;
    private int desiredPosition;
    private double pos;

    public ElevatorMovementCommand(ElevatorSubsystem l_Lifter, int desiredPosition){
        this.l_Lift = l_Lifter;
        addRequirements(l_Lift);

        this.desiredPosition = desiredPosition;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Auton State", "Begining Aimming");
        switch(desiredPosition){
            case 1:
                pos = 0;
                break;
            case 2:
                pos = 30;
                break;
            case 3:
                pos = 40;
                break;
            case 4:
                pos = 50;
                break;
            default:
                pos = 0;
                break;
        }
    }

    @Override
    public void execute() {
        // System.out.println(desiredAngle.getAsDouble());
        double currentPos = l_Lift.getEncoderValue();
        final double MAX_SPEED_RPM = 3; // Maximum speed of the motor in RPM
        final double ANGLE_TOLERANCE = 1.0;
        // Calculate the angle difference
        double posDifference = pos - currentPos;
        DifferenceOfAngle = posDifference;
        // Calculate the speed based on the angle difference
        double speedPercentage = posDifference / 180.0; // Scaling the angle difference to [-1, 1]
        double speed = (speedPercentage * MAX_SPEED_RPM);
        
        // Ensure the speed is within the motor's range
        speed = Math.min(MAX_SPEED_RPM, Math.max(-MAX_SPEED_RPM, speed));
        
        
        // If the angle difference is within the tolerance, stop the motor
        if (Math.abs(posDifference) <= ANGLE_TOLERANCE) {
            speed = 0; // Stop the motor
        } else if (Math.abs(posDifference) > 10 && Math.abs(speed) < 0.5){
            if (speed < 0) {
                speed = -0.5;
            } else if (speed > 0){
                speed = 0.5;
            }
        } else if (Math.abs(posDifference) > 0.5 && Math.abs(speed) < 0.3){
            if (speed < 0) {
                speed = -0.3;
            } else if (speed > 0){
                speed = 0.3;
            }
        }
        l_Lift.setLiftSpeed(speed);
        SmartDashboard.putNumber("Desired Movement Speed: ", speed);
    }

    @Override
    public void end(boolean interrupted) {
        l_Lift.setLiftSpeed(0);
        SmartDashboard.putString("Auton State", "Movement Complete");
    }

    @Override
    public boolean isFinished(){
        if (Math.abs(DifferenceOfAngle) <= 0.5){
            return true;
        } else{
            return false;
        }
    }
}
