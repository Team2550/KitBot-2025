package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.CoralHandlerConstants;

enum CoralHandlerPoints {
    REST(0,0),
    PICKUP(0,0),
    L1(0,0),
    L2(0,0),
    L3(0,0),
    L4(0,0);

    private double height;
    private double angle;
    CoralHandlerPoints(double height, double angle) {
        this.height = height;
        this.angle = angle;
    }

    public double getHeight() {
        return this.height;
    }

    public double getAngle() {
        return this.angle;
    }
}

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;
    private final PositionVoltage mElevatorRequest;
    private final PositionVoltage mArmRequest;
    private final Solenoid mCoralHolderSolenoid;
    

    private static final double multiplier = 7.07;
        

    public CoralHandlerSubsystem() {

        //Set Up Elevator Motor
        mElevatorMotor = new TalonFX(9);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorFXConfig);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorPIDConfig);
        
        mElevatorMotor.setPosition(0);
        mElevatorRequest = new PositionVoltage(0).withSlot(0);

        //Set Up Arm Motor
        mArmMotor = new TalonFX(10);
        mArmMotor.getConfigurator().apply(Robot.ctreConfigs.armMotorFXConfig);
        mArmMotor.getConfigurator().apply(Robot.ctreConfigs.armMotorPIDConfig);
        
        mArmMotor.setPosition(-0.25);
        //Right is 0
        //Up is 90
        //Left is 180
        //Down is 270
        mArmRequest = new PositionVoltage(90).withSlot(0);

        mCoralHolderSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

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
                selectedHeightCommand = () -> moveElevatorTo(CoralHandlerConstants.L2Pose);
                break;
            case 2:
            case 3:
                selectedHeightCommand = () -> moveElevatorTo(CoralHandlerConstants.L3Pose);
                break;
            case 4:
            case 5:
                selectedHeightCommand = () -> moveElevatorTo(CoralHandlerConstants.L4Pose);
                break;
            default:
                break;
        }
        //TODO: Check if doing automatic scoring and obtain scoring position if so
        return this.run(selectedHeightCommand).andThen(new InstantCommand(() -> mCoralHolderSolenoid.set(true)));
    }

    public Command zero() { return null; }

    //TODO: Probably come up with better naming to not confuse with the Swerve system and Pose2D's
    public void changeToIntakePose() {}

    // Sets the height of the elevator from the bottom in meters
    public Command moveElevatorTo(double height) {
        if (0 <= height && height <= 0.762) {
            return this.run(() -> {
                mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(height, 0.13) * multiplier));
            });
        } else {
            return null;
        }
    }

    public double getArmMotorPosition() {
        return mArmMotor.getPosition().getValueAsDouble();
    }

    // Returns the elevator to the bottom at its predefined resting position
    public Command rest() {
        return this.run(() -> {
            mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(CoralHandlerConstants.restPose, 0.13) * multiplier));
        });
    }

    // Stops the elevator motor at its current position
    public void stopElevatorMotor() {
        mElevatorMotor.set(0);
    }

    // Stops the arm at its current position
    public void stopArmMotor(){
        mArmMotor.set(0);
    }

    public Command changeArmPose(Rotation2d pose) {
        // mArmMotor.setControl(mArmRequest.withPosition((pose.getDegrees() * 62.5) / 360));
        if (-180 <= pose.getDegrees() && pose.getDegrees() <= 180) {
            return this.run(() -> {
                mArmMotor.setControl(mArmRequest.withPosition((pose.getDegrees()) / 360));
            });
        } else {
            return null;
        }
    }
}
