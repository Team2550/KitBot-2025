package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.lang.reflect.Array;
import java.net.SocketPermission;
import java.util.List;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;

    public CoralHandlerSubsystem() {

        // CANcoder eventually probably idk their plan

        Slot0Configs elevatorMotorConfig = new Slot0Configs();
        elevatorMotorConfig.kP = 0.12;
        elevatorMotorConfig.kI = 0;
        elevatorMotorConfig.kD = 0;
        elevatorMotorConfig.GravityType = GravityTypeValue.Elevator_Static;

        mElevatorMotor = new TalonFX(9);
        mElevatorMotor.getConfigurator().apply(elevatorMotorConfig);
        mElevatorMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits
                    .withStatorCurrentLimit(60)
                    .withSupplyCurrentLimit(30));
        
        // mArmMotor = new TalonFX(-1);
        // mArmMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit(30));

        mNetworkTable = NetworkTableInstance.getDefault();
        mScoringNetworkTable = mNetworkTable.getTable("automaticScoringPosition");
        mSelectedScoringHeightEntry = mScoringNetworkTable.getEntry("scoringHeight");
        mSelectedScoringSideEntry = mScoringNetworkTable.getEntry("scoringSide");
    }

    public Command intake() {
        return null;
    }
    public Command expel() {
        Runnable selectedHeightCommand = () -> {};
        int scoringPosition = (int)mSelectedScoringHeightEntry.getInteger(-1);
        switch (scoringPosition) {
            case 0:
            case 1:
                selectedHeightCommand = () -> changeToL2Pose();
                break;
            case 2:
            case 3:
                selectedHeightCommand = () -> changeToL3Pose();
                break;
            case 4:
            case 5:
                selectedHeightCommand = () -> changeToL4Pose();
                break;
            default:
                break;
        }
        //TODO: Check if doing automatic scoring and obtain scoring position if so
        return this.run(selectedHeightCommand);
    }

    public Command zero() { return null; }

    //TODO: Probably come up with better naming to not confuse with the Swerve system and Pose2D's
    public void changeToPickupPose() {

    }

    public void rest() {
        double gearRatio = 12/2;
        final PositionVoltage mRequest = new PositionVoltage(0).withSlot(0);
        mElevatorMotor.setControl(mRequest.withPosition(Conversions.metersToRotations(0.15, 0.1189) * gearRatio));
        // mElevatorMotor.setPosition(Conversions.metersToRotations(0.15, gearRatio * 4.6809));
    }
    // 50:1 24:1
    public void changeToL2Pose() {
        double gearRatio = 12/2;
        final PositionVoltage mRequest = new PositionVoltage(0).withSlot(0);
        mElevatorMotor.setControl(mRequest.withPosition(Conversions.metersToRotations(0.15, 0.1149) * gearRatio));
        // mElevatorMotor.setPosition(Conversions.metersToRotations(0.3, gearRatio * 4.6809));
    }
    public void stopElevatorMotor() {
        mElevatorMotor.set(0);
    }
    public void changeToL3Pose() {}
    public void changeToL4Pose() {}
}
