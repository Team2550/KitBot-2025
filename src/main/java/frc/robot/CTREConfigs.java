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

        // ELEVATOR CONFIGURATION

        elevatorMotorFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        elevatorMotorFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorMotorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorFXConfig.Feedback.SensorToMechanismRatio = 1;

        elevatorMotorPIDConfig.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorPIDConfig.kP = 0.5;
        elevatorMotorPIDConfig.kI = 0.0;
        elevatorMotorPIDConfig.kD = 0.1;
        elevatorMotorPIDConfig.kG = 0.5;

        // ARM CONFIGURATION

        armMotorFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        armMotorFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        armMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        armMotorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotorFXConfig.Feedback.SensorToMechanismRatio = 62.5;

        armMotorPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
        armMotorPIDConfig.kP = 24;
        armMotorPIDConfig.kI = 0.05;
        armMotorPIDConfig.kD = 0.1;
        armMotorPIDConfig.kG = -1.375;
    }
}
