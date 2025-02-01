package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants.CoralHandlerConstants;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;
    private final PositionVoltage mElevatorRequest;
    private final PositionVoltage mArmRequest;
    

    private final double multiplier;
        

    public CoralHandlerSubsystem() {
        multiplier = 7.05;



        //Set Up Elevator Motor
        mElevatorMotor = new TalonFX(9);
        Slot0Configs elevatorMotorConfig = new Slot0Configs();
        elevatorMotorConfig.kP = 0.5;
        elevatorMotorConfig.kI = 0; 
        elevatorMotorConfig.kV = 0.1;
        elevatorMotorConfig.kG = 0.5;
        elevatorMotorConfig.GravityType = GravityTypeValue.Elevator_Static; 
        mElevatorMotor.getConfigurator().apply(elevatorMotorConfig);
        mElevatorMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits
                    .withStatorCurrentLimit(120)
                    .withSupplyCurrentLimit(40));
        
        mElevatorMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive));
        mElevatorMotor.setPosition(0);
        mElevatorMotor.getConfigurator().apply(new TalonFXConfiguration().Feedback.withSensorToMechanismRatio(1));
        mElevatorRequest = new PositionVoltage(0).withSlot(0);



        //Set Up Arm Motor
        mArmMotor = new TalonFX(10);
        Slot0Configs armMotorConfig = new Slot0Configs();
        armMotorConfig.kP = 0.3;
        armMotorConfig.kI = 0.0; 
        armMotorConfig.kV = 0.0;
        armMotorConfig.kG = 0.5;
        armMotorConfig.GravityType = GravityTypeValue.Arm_Cosine; 
        mArmMotor.getConfigurator().apply(armMotorConfig);
        mArmMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits
                    .withStatorCurrentLimit(120)
                    .withSupplyCurrentLimit(40));
        
        mArmMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive));
        mArmMotor.setPosition(-0.25);
        mArmMotor.getConfigurator().apply(new TalonFXConfiguration().Feedback.withSensorToMechanismRatio(1));
        //Right is 0
        //Up is 90
        //Left is 180
        //Down is 270
        mArmRequest = new PositionVoltage(90).withSlot(0);





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
                selectedHeightCommand = () -> changeToPose(CoralHandlerConstants.L2Pose);
                break;
            case 2:
            case 3:
                selectedHeightCommand = () -> changeToPose(CoralHandlerConstants.L3Pose);
                break;
            case 4:
            case 5:
                selectedHeightCommand = () -> changeToPose(CoralHandlerConstants.L4Pose);
                break;
            default:
                break;
        }
        //TODO: Check if doing automatic scoring and obtain scoring position if so
        return this.run(selectedHeightCommand);
    }

    public Command zero() { return null; }

    //TODO: Probably come up with better naming to not confuse with the Swerve system and Pose2D's
    public void changeToPickupPose() {}

    public void changeToPose(double pose) {
        if (0 <= pose && pose <= 0.762) {
            mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(pose, 0.13) * multiplier));
        } else {
            return;
        }
    }
    public void rest() {
        mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(CoralHandlerConstants.restPose, 0.13) * multiplier));
    }
    public void stopElevatorMotor() {
        mElevatorMotor.set(0);
    }

    public void stopArmMotor(){
        mArmMotor.set(0);
    }

    public void changeArmPose(Rotation2d pose) {
        mArmMotor.setControl(mArmRequest.withPosition((pose.getDegrees() * 62.5) / 360));
        if (-180 <= pose.getDegrees() && pose.getDegrees() <= 180) {
            mArmMotor.setControl(mArmRequest.withPosition((pose.getDegrees() * 62.5) / 360));
        } else {
            return;
        }
    }
}
