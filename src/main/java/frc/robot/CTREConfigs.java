package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs {
    public TalonFXConfiguration elevatorMotorFXConfig = new TalonFXConfiguration();
    public Slot0Configs elevatorMotorPIDConfig = new Slot0Configs();
    public TalonFXConfiguration armMotorFXConfig = new TalonFXConfiguration();
    public Slot0Configs armMotorPIDConfig = new Slot0Configs();

    public CTREConfigs() {
        elevatorMotorFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        elevatorMotorFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorMotorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorFXConfig.Feedback.SensorToMechanismRatio = 1;

        elevatorMotorPIDConfig.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorPIDConfig.kP = 0.3;
        elevatorMotorPIDConfig.kI = 0;
        elevatorMotorPIDConfig.kD = 0.1;
        elevatorMotorPIDConfig.kG = 0.5;
    }
}
