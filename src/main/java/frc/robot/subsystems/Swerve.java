package frc.robot.subsystems;

import frc.lib.util.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AutoAlignmentSubsystem.pose;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final SwerveDrivePoseEstimator m_PoseEstimator;
    
    public Swerve() {
        //positional logging
        Pose2d poseA = getPose();
        Pose2d poseB = new Pose2d();    

        Logger.recordOutput("MyPose", poseA);
        Logger.recordOutput("MyPoseArray", poseA, poseB);
        Logger.recordOutput("MyPoseArray", new Pose2d[] {poseA, poseB});
        
        //gyro initialization
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Swerve");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        //initialization of position estimation
        m_PoseEstimator  = 
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            getGyroYaw(), 
            getModulePositions(), 
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        
        //reserve swerve odometry initialization
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        //setup of auto builder for pathplanner
        try{
            Constants.AutoConstants.config = RobotConfig.fromGUISettings();
      
            System.out.println("Configured Auto Builder");

            // Configure AutoBuilder
            AutoBuilder.configure(
              this::getPose, 
              this::setPose, 
              this::getRobotRelativeSpeeds, 
              (speeds, feedforwards) -> driveRobotRelative(speeds), 
              new PPHolonomicDriveController(
                new PIDConstants(10.0,0.0,0.0),
                new PIDConstants(7.0, 0.0, 0.0)
              ),
              Constants.AutoConstants.config,
              () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
              },
              this
            );
          }catch(Exception e){
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
          }
    }

    public void updateOdometry() {
        //update estimated position with swerve odometry 
        m_PoseEstimator.update(
            getGyroYaw(), 
            getModulePositions());

        boolean doRejectUpdate = false;
        try{
            if (TrackingSubsystem.left_id != -1){
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
      
                if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
                {
                    if(mt1.rawFiducials[0].ambiguity > .7)
                    {
                    doRejectUpdate = true;
                    }
                    if(mt1.rawFiducials[0].distToCamera > 3)
                    {
                    doRejectUpdate = true;
                    }
                }
                if(mt1.tagCount == 0)
                {
                    doRejectUpdate = true;
                }

                if(!doRejectUpdate)
                {
                    m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
                    m_PoseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
                }



                // LimelightHelpers.SetRobotOrientation("limelight-left", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                // LimelightHelpers.PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
                // if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                // {
                //     doRejectUpdate = true;
                // }
                // if(mt.tagCount == 0)
                // {
                //     doRejectUpdate = true;
                // }
                // if(!doRejectUpdate)
                // {
                //     m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                //     m_PoseEstimator.addVisionMeasurement(
                //         mt.pose,
                //         mt.timestampSeconds);
                // }
            }

            if (TrackingSubsystem.right_id != -1){
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
      
                if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
                {
                    if(mt1.rawFiducials[0].ambiguity > .7)
                    {
                    doRejectUpdate = true;
                    }
                    if(mt1.rawFiducials[0].distToCamera > 3)
                    {
                    doRejectUpdate = true;
                    }
                }
                if(mt1.tagCount == 0)
                {
                    doRejectUpdate = true;
                }

                if(!doRejectUpdate)
                {
                    m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
                    m_PoseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
                }

                // LimelightHelpers.SetRobotOrientation("limelight-right", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                // LimelightHelpers.PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
                // if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                // {
                //     doRejectUpdate = true;
                // }
                // if(mt.tagCount == 0)
                // {
                //     doRejectUpdate = true;
                // }
                // if(!doRejectUpdate)
                // {
                //     m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                //     m_PoseEstimator.addVisionMeasurement(
                //         mt.pose,
                //         mt.timestampSeconds);
                // } 
            }
        } catch(NullPointerException e){

        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        try{
            return m_PoseEstimator.getEstimatedPosition();
        }catch(NullPointerException n){
            try {
                System.out.println("base swerve odometry");
                return swerveOdometry.getPoseMeters();
            } catch(NullPointerException s){
                System.out.println("Default Pose");    
                return new Pose2d(3.213, 4.025, new Rotation2d(-90));
            }
        }
    }

    public void setPose(Pose2d pose) {
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    Pose2d targetPose = new Pose2d();
    public Command driveToPose(TrackingSubsystem t_Tracking, AutoAlignmentSubsystem a_Alignment, int offsetDirection){
        Command setDesiredPose = this.run(
            () -> targetPose = a_Alignment.basePose(t_Tracking.validId, t_Tracking.isLeft, offsetDirection, getPose())
        );
        WaitCommand wait = new WaitCommand(0.1);
        //obtain base position to move to before offsets
        // Pose2d targetPose = a_Alignment.basePose(t_Tracking.validId, t_Tracking.isLeft, offsetDirection, getPose());
        // System.out.println("Desired Pose: " + a_Alignment.basePose(t_Tracking.validId, t_Tracking.isLeft, offsetDirection, getPose()));
        // Pose2d targetPose = new Pose2d(3, 4, new Rotation2d(-90));
        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0// Goal end velocity in meters/sec
        );
        // System.out.println("Returning pathfinding");
        return (setDesiredPose.raceWith(wait)).andThen(pathfindingCommand);
    }


    @Override
    public void periodic(){
        updateOdometry();
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
        SmartDashboard.putNumber("Yaw ", gyro.getYaw().getValueAsDouble());
    }
}