// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AlgaePivotCommand extends Command {
//     public AlgaePivotCommand(){

//     }

//     @Override
//     public void execute() {
//         // System.out.println(desiredAngle.getAsDouble());
//         double currentPos = 0;
//         final double MAX_SPEED_RPM = 3; // Maximum speed of the motor in RPM
//         final double ANGLE_TOLERANCE = 1.0;
//         // Calculate the angle difference
//         double posDifference = pos - currentPos;
//         DifferenceOfAngle = posDifference;
//         // Calculate the speed based on the angle difference
//         double speedPercentage = posDifference / 180.0; // Scaling the angle difference to [-1, 1]
//         double speed = (speedPercentage * MAX_SPEED_RPM);
        
//         // Ensure the speed is within the motor's range
//         speed = Math.min(MAX_SPEED_RPM, Math.max(-MAX_SPEED_RPM, speed));
        
        
//         // If the angle difference is within the tolerance, stop the motor
//         if (Math.abs(posDifference) <= ANGLE_TOLERANCE) {
//             speed = 0; // Stop the motor
//         } else if (Math.abs(posDifference) > 10 && Math.abs(speed) < 0.5){
//             if (speed < 0) {
//                 speed = -0.5;
//             } else if (speed > 0){
//                 speed = 0.5;
//             }
//         } else if (Math.abs(posDifference) > 0.5 && Math.abs(speed) < 0.3){
//             if (speed < 0) {
//                 speed = -0.3;
//             } else if (speed > 0){
//                 speed = 0.3;
//             }
//         }
//         l_Lift.setLiftSpeed(speed);
//         SmartDashboard.putNumber("Desired Movement Speed: ", speed);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         l_Lift.setLiftSpeed(0);
//         SmartDashboard.putString("Auton State", "Movement Complete");
//     }

//     @Override
//     public boolean isFinished(){
//         if (Math.abs(DifferenceOfAngle) <= 0.5){
//             return true;
//         } else{
//             return false;
//         }
//     }
// }
