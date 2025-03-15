package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAlignmentSubsystem extends SubsystemBase {
    enum pose{
        ID0() {
            @Override
            public int rotation(boolean isLeft) {
                // TODO Auto-generated method stub
                return 0;
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID6(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return 30;
                } else {
                    return -150;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID7(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return 90;
                } else {
                    return -90;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID8(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return 150;
                } else {
                    return -30;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID9(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -150;
                } else {
                    return 30;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID10(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -90;
                } else {
                    return 90;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID11(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -30;
                } else {
                    return 150;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },

        ID17(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return 30;
                } else {
                    return -150;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID18(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return 90;
                } else {
                    return -90;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                double[] offset = {};
                if(isLeft){
                    if (pose2 == 1){
                        offset[0] = 0.1;
                    } else {
                        offset[0] = 0.425;
                    }
                    return offset;
                } else {
                    if (pose2 == 1){
                        offset[0] = -0.425;
                    } else {
                        offset[0] = -0.1;
                    }
                    return offset;
                }
            }
        },
        ID19(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -150;
                } else {
                    return 30;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID20(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -90;
                } else {
                    return 90;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID21(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -30;
                } else {
                    return 150;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        },
        ID22(){
            @Override
            public int rotation(boolean isLeft) {
                if(isLeft){
                    return -90;
                } else {
                    return 90;
                }
            }
            @Override
            public double[] applyOffset(boolean isLeft, int pose2) {
                // TODO Auto-generated method stub
                return null;
            }
        };
        public abstract int rotation(boolean isLeft); 
        public abstract double[] applyOffset(boolean isLeft, int pose2); 
    }

    static Pose2d targetPose;
    
        public Pose2d basePose(pose pose, boolean isLeft, int offsetDirection, Pose2d defaultPose){
            switch(pose){
                //Red Alliance
                case ID6:
                    targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID7:
                targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID8:
                targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID9:
                targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID10:
                targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID11:
                targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;

            //Blue Aliance
            case ID17:
                targetPose = new Pose2d(3.830, 2.920, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID18:
                // double[] offset = pose.applyOffset(isLeft, offsetDirection);
                System.out.println("Id 18 Pose");
                targetPose = new Pose2d(3.213, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID19:
                targetPose = new Pose2d(3.830, 5.130, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID20:
                targetPose = new Pose2d(5.150, 5.130, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID21:
                targetPose = new Pose2d(5.767, 4.025, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            case ID22:
                targetPose = new Pose2d(5.150, 2.920, Rotation2d.fromDegrees(pose.rotation(isLeft)));
                return targetPose;
            default:
                targetPose = defaultPose;
                return defaultPose;
        }
    }

    @Override
    public void periodic() {
        try{
            SmartDashboard.putNumber("Desired Pose X", targetPose.getTranslation().getX());
            SmartDashboard.putNumber("Desired Pose Y", targetPose.getTranslation().getY());
            SmartDashboard.putNumber("Desired Pose Rot", targetPose.getRotation().getDegrees());
        }catch(NullPointerException n){
            // System.out.println("Unable to read target pose");
        }
    }
}
