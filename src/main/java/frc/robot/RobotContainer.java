package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.COREConstants;
import frc.lib.util.Logitech;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants;
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

    // private final JoystickButton offsetLeft = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton offsetRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Operator Controls */
    private final COREConstants pivotSpeed = Constants.AlgaeConstants.kHoldSpeed;


    /* Operator Buttons */
    private final POVButton elevatorStage0 = new POVButton(operator, 180);
    private final POVButton elevatorStage1 = new POVButton(operator, 270);
    private final POVButton elevatorStage2 = new POVButton(operator, 90);
    private final POVButton elevatorStage3 = new POVButton(operator, 0);

    private final JoystickButton closeGripper = new JoystickButton(operator, Logitech.Button.kY.value);
    private final JoystickButton openGripper = new JoystickButton(operator, Logitech.Button.kX.value);
    private final JoystickButton runFeedRight = new JoystickButton(operator, Logitech.Button.kRightBumper.value);
    private final JoystickButton runFeedLeft = new JoystickButton(operator, Logitech.Button.kLeftBumper.value);
    
    private final JoystickButton elevatorUp = new JoystickButton(operator, Logitech.Button.kA.value);
    private final JoystickButton elevatorDown = new JoystickButton(operator, Logitech.Button.kB.value);

    private final JoystickButton algaeIntake = new JoystickButton(operator, Logitech.Button.kLeftTrigger.value);
    private final JoystickButton algaeOutake = new JoystickButton(operator, Logitech.Button.kRightTrigger.value);    
    private final JoystickButton algaePivotRight = new JoystickButton(operator, Logitech.Button.kBack.value);    
    private final JoystickButton algaePivotLeft = new JoystickButton(operator, Logitech.Button.kStart.value);    

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final HolderSubsystem h_Holder = new HolderSubsystem();
    private final ElevatorSubsystem e_Elevator = new ElevatorSubsystem();
    private final TrackingSubsystem t_Tracking = new TrackingSubsystem();
    private final AutoAlignmentSubsystem a_Alignment = new AutoAlignmentSubsystem();
    private final AlgaeSubsystem al_Algae = new AlgaeSubsystem();

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

        h_Holder.setGripperPosition(1300);

        autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        SmartDashboard.putData("Auton", autoChooser);
    }
    
    /**
     * Use this method to register your commands for Autos. Commands registered for PathPlanner will not automatically
     * apear as you will have to manually enter the key value for the command.
     */
    private void registerCommands(){
        NamedCommands.registerCommand("Pause Movement", new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> 0, () -> false));
        
        NamedCommands.registerCommand("Reef Stage 1", new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage1)));
        NamedCommands.registerCommand("Reef Stage 2", new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage2)));
        NamedCommands.registerCommand("Reef Stage 3", new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage3)));
        NamedCommands.registerCommand("Reef Stage 4", new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage4)));

        NamedCommands.registerCommand("Run Feed Left", new ReleaseGripperFeedCommand(h_Holder, 1));
        NamedCommands.registerCommand("Run Feed Right", new ReleaseGripperFeedCommand(h_Holder, -1));

        NamedCommands.registerCommand("Grip", new InstantCommand(() -> h_Holder.setGripperPosition(2)));
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

        // offsetLeft.whileTrue(s_Swerve.driveToPose(t_Tracking, a_Alignment, 1));
        // offsetRight.whileTrue(s_Swerve.driveToPose(t_Tracking, a_Alignment, 2));

        /* Operator Buttons */
        elevatorStage0.onTrue(new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage1)));
        elevatorStage1.onTrue(new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage2)));
        elevatorStage2.onTrue(new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage3)));
        elevatorStage3.onTrue(new InstantCommand(() -> e_Elevator.moveToHeight(ElevatorConstants.kReefStage4))); 
        // elevatorStage0.and(elevatorStage1).and(elevatorStage2).and(elevatorStage3).onFalse(new InstantCommand(() -> e_Elevator.setLiftSpeed(0.0)));
        // elevatorStage0.and(elevatorStage1).and(elevatorStage2).and(elevatorStage3).onFalse(new InstantCommand(() -> e_Elevator.holdHeight()));
    
        closeGripper.onTrue(new InstantCommand(() -> h_Holder.setGripperPosition(2)));
        openGripper.onTrue(new InstantCommand(() -> h_Holder.setGripperPosition(1)));
        runFeedRight.whileTrue(new ReleaseGripperFeedCommand(h_Holder, -1));
        runFeedLeft.whileTrue(new ReleaseGripperFeedCommand(h_Holder, 1));

        elevatorUp.onTrue(new InstantCommand(() -> e_Elevator.setLiftSpeed(Constants.ElevatorConstants.kElevatorSpeed.get(0.0))))
                  .onFalse(new InstantCommand(() -> e_Elevator.setLiftSpeed(-0.00)));
        elevatorDown.onTrue(new InstantCommand(() -> e_Elevator.setLiftSpeed(-Constants.ElevatorConstants.kElevatorSpeed.get(0.0))))
                    .onFalse(new InstantCommand(() -> e_Elevator.setLiftSpeed(-0.00)));

        algaeIntake.onTrue(new InstantCommand(() -> al_Algae.setAlgaeMotorSpeed(0.6)))
                   .onFalse(new InstantCommand(() -> al_Algae.setAlgaeMotorSpeed(0.0)));

        algaeOutake.onTrue(new InstantCommand(() -> al_Algae.setAlgaeMotorSpeed(-0.6)))
                   .onFalse(new InstantCommand(() -> al_Algae.setAlgaeMotorSpeed(0.0)));
        
        algaePivotLeft.onTrue(new InstantCommand(() -> al_Algae.setAlgaePivotMotorSpeed(0.4)))
                      .onFalse(new InstantCommand(() -> al_Algae.setAlgaePivotMotorSpeed(pivotSpeed)));
        algaePivotRight.onTrue(new InstantCommand(() -> al_Algae.setAlgaePivotMotorSpeed(-0.4)))
                      .onFalse(new InstantCommand(() -> al_Algae.setAlgaePivotMotorSpeed(pivotSpeed)));
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
