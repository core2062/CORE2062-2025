package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.constants.Constants.ElevatorConstants;

public class ElevatorConfigs {
    public TalonFXConfiguration motorConfig;

    public ElevatorConfigs(int motor){
        motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = ElevatorConstants.kP;
        motorConfig.Slot0.kP = ElevatorConstants.kI;
        motorConfig.Slot0.kP = ElevatorConstants.kD;
        motorConfig.Slot0.kP = ElevatorConstants.kF;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.kSupplyCurrentEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.kSupplyCurrentLowerLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.kSupplyCurrentLowerTime;

        motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ElevatorConstants.kOpenLoopRamp;
        motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ElevatorConstants.kOpenLoopRamp;

        motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ElevatorConstants.kClosedLoopRamp;
        motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ElevatorConstants.kClosedLoopRamp;

        if (motor == 1){
            motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
    }
}
