package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilderException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton elvatorMotorRun = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Operator Buttons */
    private final POVButton elevatorStage0 = new POVButton(operator, 0);
    private final POVButton elevatorStage1 = new POVButton(operator, 90);
    private final POVButton elevatorStage2 = new POVButton(operator, 180);
    private final POVButton elevatorStage3 = new POVButton(operator, 270);

    private final JoystickButton closeGripper = new JoystickButton(operator, 1);
    private final JoystickButton openGripper = new JoystickButton(operator, 3);
    private final JoystickButton runFeed1 = new JoystickButton(operator, 6);
    private final JoystickButton runFeed2 = new JoystickButton(operator, 2);
    private final JoystickButton tiltLeft = new JoystickButton(operator, 4);
    private final JoystickButton tiltRight = new JoystickButton(operator, 5);
    private final JoystickButton tiltCenter = new JoystickButton(operator, 7);
    
    private final JoystickButton elevatorUp = new JoystickButton(operator, 8);
    private final JoystickButton elevatorDown = new JoystickButton(operator, 9);
    private final JoystickButton resetElevatorEncoder = new JoystickButton(operator, 10);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem a_Arm = new ArmSubsystem();
    private final HolderSubsystem h_Holder = new HolderSubsystem();
    private final ElevatorSubsystem e_Elevator = new ElevatorSubsystem();
    private final SwerveTrackingSubsystem st_SwerveTrackSubsystem = new SwerveTrackingSubsystem();
    private final ObjectTrackingSubsystem ob_ObjectTrackingSubsystem = new ObjectTrackingSubsystem();

    /* double Suppliers */

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Swerve.gyro.setYaw(0);
        // Configure the button bindings
        configureButtonBindings();

        registerCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        SmartDashboard.putData("Auton", autoChooser);
    }
    
    /**
     * Use this method to register your commands for Autos. Commands registered for PathPlanner will not automatically
     * apear as you will have to manually enter the key value for the command.
     */
    private void registerCommands(){
        NamedCommands.registerCommand("Pause Movement", new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> 0, () -> false));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        elvatorMotorRun.onTrue(new InstantCommand(() -> e_Elevator.setLiftSpeed(Constants.ElevatorConstants.kElevatorSpeed.get(0.0))))
                        .onFalse(new InstantCommand(() -> e_Elevator.setLiftSpeed(0.0)));

        /* Operator Buttons */
        elevatorStage0.onTrue(e_Elevator.elevatorLift(1));
        elevatorStage1.onTrue(e_Elevator.elevatorLift(2));
        elevatorStage2.onTrue(e_Elevator.elevatorLift(3));
        elevatorStage3.onTrue(e_Elevator.elevatorLift(4));
    
        closeGripper.onTrue(new InstantCommand(() -> h_Holder.setGripperPosition(20)));
        openGripper.onTrue(new InstantCommand(() -> h_Holder.setGripperPosition(80)));
        runFeed1.onTrue(new InstantCommand(() -> h_Holder.runBelt(0.2)))
                .onFalse(new InstantCommand(() -> h_Holder.runBelt(0)));
        runFeed2.onTrue(new InstantCommand(() -> h_Holder.runBelt(-0.2)))
                .onFalse(new InstantCommand(() -> h_Holder.runBelt(0)));
        tiltLeft.onTrue(a_Arm.rotateArm(1));
        tiltCenter.onTrue(a_Arm.rotateArm(2));
        tiltRight.onTrue(a_Arm.rotateArm(3));

        elevatorUp.onTrue(new InstantCommand(() -> e_Elevator.setLiftSpeed(-Constants.ElevatorConstants.kElevatorSpeed.get(0.0))))
                  .onFalse(new InstantCommand(() -> e_Elevator.setLiftSpeed(-0.00)));
        elevatorDown.onTrue(new InstantCommand(() -> e_Elevator.setLiftSpeed(Constants.ElevatorConstants.kElevatorSpeed.get(0.0))))
                    .onFalse(new InstantCommand(() -> e_Elevator.setLiftSpeed(-0.00)));
        resetElevatorEncoder.onTrue(new InstantCommand(() -> e_Elevator.resetEncoder()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
    
}
