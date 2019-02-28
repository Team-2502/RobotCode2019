package com.team2502.robot2019.command.autonomous.ingredients;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This could be helpful: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/DriveStraight_AuxPigeon/src/main/java/frc/robot/Constants.java
 */

public class TurnToHeadingCommand extends Command
{

    private double targetAngle, currentAngle, baseSpeed;


    public TurnToHeadingCommand(double targetAngle, double baseSpeed)
    {
        this.targetAngle = targetAngle;
        this.baseSpeed = baseSpeed;
    }

    @Override
    protected void initialize()
    {
        //Disable Talons
        Robot.DRIVE_TRAIN.getFrontRight().set(ControlMode.PercentOutput, 0);
        Robot.DRIVE_TRAIN.getFrontLeft().set(ControlMode.PercentOutput, 0);

        Robot.DRIVE_TRAIN.getFrontRight().configFactoryDefault();
        Robot.DRIVE_TRAIN.getFrontLeft().configFactoryDefault();

        Robot.DRIVE_TRAIN.getFrontRight().setNeutralMode(NeutralMode.Brake);
        Robot.DRIVE_TRAIN.getFrontLeft().setNeutralMode(NeutralMode.Brake);

        //Config sensor feedback
        Robot.DRIVE_TRAIN.getFrontRight().configRemoteFeedbackFilter(Robot.PIGEON.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, 0);
        Robot.DRIVE_TRAIN.getFrontRight().configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);// pidIx for aux PigeonPID index
        Robot.DRIVE_TRAIN.getFrontRight().configSelectedFeedbackCoefficient(Constants.Autonomous.PigeonPID.TURN_TRAVEL_UNITS_PER_ROTATION/ Constants.Autonomous.PigeonPID.PIGEON_UNITS_PER_ROTATION, 1, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        //Config output & sensor direction
        Robot.DRIVE_TRAIN.getFrontRight().setSensorPhase(true); // Must be set before setInverted()
        Robot.DRIVE_TRAIN.getFrontLeft().setSensorPhase(true);
        Robot.DRIVE_TRAIN.getFrontRight().setInverted(false); // TODO Change if necessary
        Robot.DRIVE_TRAIN.getFrontLeft().setInverted(false);

        Robot.DRIVE_TRAIN.getFrontRight().setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.PIGEON.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,5, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        Robot.DRIVE_TRAIN.getFrontRight().configNeutralDeadband(0.001, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontLeft().configNeutralDeadband(0.001, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        //Config peak outputs
        Robot.DRIVE_TRAIN.getFrontRight().configPeakOutputForward(+1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontLeft().configPeakOutputForward(+1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().configPeakOutputReverse(-1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontLeft().configPeakOutputReverse(-1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        //Set turn constants
        Robot.DRIVE_TRAIN.getFrontRight().config_kP(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, Constants.Autonomous.PigeonPID.TURNING_GAINS[0], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().config_kI(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, Constants.Autonomous.PigeonPID.TURNING_GAINS[1], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().config_kD(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, Constants.Autonomous.PigeonPID.TURNING_GAINS[2], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().config_kF(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, baseSpeed, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.DRIVE_TRAIN.getFrontRight().config_IntegralZone(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, (int) Constants.Autonomous.PigeonPID.TURNING_GAINS[3], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS); //
        Robot.DRIVE_TRAIN.getFrontRight().configAllowableClosedloopError(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, 0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS); // 0 Is an ID

    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.getFrontRight().selectProfileSlot(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, 1);
        Robot.DRIVE_TRAIN.getFrontRight().set(ControlMode.PercentOutput, Constants.Autonomous.PigeonPID.TURNING_GAINS[4], DemandType.AuxPID, targetAngle);
        Robot.DRIVE_TRAIN.getFrontLeft().set(ControlMode.PercentOutput, -Robot.DRIVE_TRAIN.getFrontRight().getMotorOutputPercent());
        currentAngle = Robot.PIGEON.getAbsoluteCompassHeading(); // getAbsoluteCompassHeading returns a double 0-360
    }

    @Override
    protected void end()
    {
        Robot.DRIVE_TRAIN.runMotorsVelocity(0,0);
        Robot.DRIVE_TRAIN.getFrontRight().configFactoryDefault();
        Robot.DRIVE_TRAIN.getFrontLeft().configFactoryDefault();
    }

    @Override
    protected boolean isFinished(){
        // Check for if current angle is within error threshold (extra logic for if currentAngle and targetAngle
        //                                                       are on opposite sides (ie. cA == 0 && tA == 360))
        if(((currentAngle<=targetAngle+Constants.Autonomous.PigeonPID.MAX_ERR_THRESHOLD)
                &&(currentAngle>=targetAngle-Constants.Autonomous.PigeonPID.MAX_ERR_THRESHOLD))
                || currentAngle<=targetAngle-360+Constants.Autonomous.PigeonPID.MAX_ERR_THRESHOLD
                || currentAngle>=targetAngle+360-Constants.Autonomous.PigeonPID.MAX_ERR_THRESHOLD)
        {
        return true;
        }
        return false;
    }

}
