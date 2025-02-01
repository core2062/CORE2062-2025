package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.constants.Constants.ElevatorConstants;

public class ElevatorConfigs {
    public TalonSRXConfiguration motorConfig;

    public ElevatorConfigs(){
        motorConfig = new TalonSRXConfiguration();
        motorConfig.slot0.kP = ElevatorConstants.kP;
        motorConfig.slot0.kI = ElevatorConstants.kI;
        motorConfig.slot0.kD = ElevatorConstants.kD;
        motorConfig.slot0.kF = ElevatorConstants.kF;

        motorConfig.continuousCurrentLimit = Constants.ElevatorConstants.motorContinuousCurrentLimit;
        motorConfig.peakCurrentDuration = Constants.ElevatorConstants.motorPeakCurrentDuration;
        motorConfig.peakCurrentLimit = Constants.ElevatorConstants.motorPeakCurrentLimit;
        motorConfig.openloopRamp = Constants.ElevatorConstants.kOpenloopRamp;
        motorConfig.closedloopRamp = Constants.ElevatorConstants.kClosedloopRamp;
        motorConfig.motionAcceleration = 20.4;
        motorConfig.motionCruiseVelocity = 24.8;
        motorConfig.motionCurveStrength = 9;
    }
}
