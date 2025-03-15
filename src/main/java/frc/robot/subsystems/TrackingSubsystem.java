package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AutoAlignmentSubsystem.pose;

public class TrackingSubsystem extends SubsystemBase {

    Set<Double> ReefValidTags = new HashSet<Double>(Arrays.asList(
        new Double[] {17.0,18.0,19.0,20.0,21.0,22.0}
    ));

    //Blue side : 17.0, 18.0, 19.0, 20.0, 21.0, 22.0
    //Red side : 6.0, 7.0, 8.0, 9.0, 10.0, 11.0

    // double angle;
    public double left_x ;
    public static double left_id ;
    double left_y ;
    double left_area ;

    public double right_x ;
    public static double right_id ;
    double right_y ;
    double right_area ;

    double delta ; 
  
    boolean isLeft;
    pose validId = pose.ID0;

    NetworkTable table_Left = NetworkTableInstance.getDefault().getTable("limelight-left");
    NetworkTableEntry left_tx = table_Left.getEntry("tx");
    NetworkTableEntry left_ty = table_Left.getEntry("ty");
    NetworkTableEntry left_ta = table_Left.getEntry("ta");
    NetworkTableEntry left_tid = table_Left.getEntry("tid");
    NetworkTableEntry left_pipeline = table_Left.getEntry("pipeline");

    NetworkTable right_table = NetworkTableInstance.getDefault().getTable("limelight-right");
    NetworkTableEntry right_tx = right_table.getEntry("tx");
    NetworkTableEntry right_ty = right_table.getEntry("ty");
    NetworkTableEntry right_ta = right_table.getEntry("ta");
    NetworkTableEntry right_tid = right_table.getEntry("tid");
    NetworkTableEntry right_pipeline = right_table.getEntry("pipeline");

    public TrackingSubsystem(){
    }

    @Override
    public void periodic() {
        left_x = left_tx.getDouble(0.0);
        left_id = left_tid.getDouble(0.0);
        left_y = left_ty.getDouble(0.0);
        left_area = left_ta.getDouble(0.0);

        right_x = right_tx.getDouble(0.0);
        right_id = right_tid.getDouble(0.0);
        right_y = right_ty.getDouble(0.0);
        right_area = right_ta.getDouble(0.0);

        SmartDashboard.putNumber("left_limelightx", left_x);
        SmartDashboard.putNumber("left_limelighty", left_y);
        SmartDashboard.putNumber("left_limelighta", left_area);
        SmartDashboard.putNumber("left_limelightid", left_id);

        SmartDashboard.putNumber("right_limelightx", right_x);
        SmartDashboard.putNumber("right_limelighty", right_y);
        SmartDashboard.putNumber("right_limelighta", right_area);
        SmartDashboard.putNumber("right_limelightid", right_id);

        if (ReefValidTags.contains(left_id)){
            isLeft = true;
        } else if (ReefValidTags.contains(right_id)){
            isLeft = false;
        }
        if (left_id != -1 || right_id != -1){
            validId = getValidIds();
        }
    }

    public pose getValidIds() {
        double tv;
        if(isLeft){
            tv = left_id;
        } else {
            tv = right_id;
        }
        switch((int) tv) {
            case 6:
                return pose.ID6;
            case 7:
                return pose.ID7;
            case 8:
                return pose.ID8;
            case 9:
                return pose.ID9;
            case 10:
                return pose.ID10;
            case 11:
                return pose.ID11;
            case 17:
                return pose.ID17;
            case 18:
                return pose.ID18;
            case 19:
                return pose.ID19;
            case 20:
                return pose.ID20;
            case 21:
                return pose.ID21;
            case 22:
                return pose.ID22;
            default:
                return null;
        }
    }
    
    // public Command AimAtSpeaker(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
    //     System.out.println(Constants.VisionConstants.SpeakerID);
    //     Command setPipelineCommand = this.run(
    //         () -> pipeline.setDouble(Constants.VisionConstants.SpeakerID)
    //         );
    //         setPipelineCommand.addRequirements(this);
    //         Command rotateSwerveCommand = new TeleopSwerve(
    //             s_Swerve,
    //             translationSup,
    //             strafeSup,
    //             () -> getRotation(0),
    //             robotCentricSup 
    //         );
    //     return setPipelineCommand.alongWith(rotateSwerveCommand);
    // }

    // public void setPipelineSpeaker(){
    //     pipeline.setDouble(Constants.VisionConstants.SpeakerID);
    // }
    
    // public double getDistance(){
    //     double area = ta.getDouble(0.0);
    //     double oneSide = Math.sqrt(area);
    //     double distance = 4.83/oneSide; 
    //     return distance;
    // }

    // public double getTranslation(double targetDistance){
    //     double distance = getDistance();
    //     if (Double.isInfinite(distance)){
    //         return 0;
    //     }
    //     else{
    //         return targetDistance-distance;
    //     }
    // }

    // public double getRotation(double targetAngle){
    //     // System.out.println("id: " + id);
    //     //adjusting for mounting angle offset
    //     targetAngle += 2;
    //     if (id <= 0){
    //         System.out.println("id is 0");
    //         return 0;
    //     }
    //     else{
    //         if (tx.getDouble(0.0) > (targetAngle+10)){
    //             return (tx.getDouble(0.0)-targetAngle)*-0.03;
    //         } else  if (tx.getDouble(0.0) > (targetAngle+5)){
    //             return (tx.getDouble(0.0)-targetAngle)*-0.04;
    //         } else {
    //             return (tx.getDouble(0.0)-targetAngle)*-0.05;
    //         }
    //     }
    // }

        // public double getRotationAuton(double targetAngle){
        // // System.out.println("id: " + id);
        // //adjusting for mounting angle offset
        // targetAngle += 2;
        // if (id <= 0){
        //     System.out.println("id is 0");
        //     return 0;
        // }
        // else{
        //     delta = tx.getDouble(0.0) - targetAngle;
        //     if (Math.abs(delta) > 10){
        //         return delta*-0.02;
        //     } else  if (Math.abs(delta) > 5){
        //         return delta*-0.035;
        //     } else {
        //         return delta*-0.04;
        //     }
        // }
    // }
}