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
        Robot.RIGHT_SIDE.set(ControlMode.PercentOutput, 0);
        Robot.LEFT_SIDE.set(ControlMode.PercentOutput, 0);

        Robot.RIGHT_SIDE.configFactoryDefault();
        Robot.LEFT_SIDE.configFactoryDefault();

        Robot.RIGHT_SIDE.setNeutralMode(NeutralMode.Brake);
        Robot.LEFT_SIDE.setNeutralMode(NeutralMode.Brake);

        //Config sensor feedback
        Robot.RIGHT_SIDE.configRemoteFeedbackFilter(Robot.PIGEON.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, 0);
        Robot.RIGHT_SIDE.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);// pidIx for aux PigeonPID index
        Robot.RIGHT_SIDE.configSelectedFeedbackCoefficient(Constants.Autonomous.PigeonPID.TURN_TRAVEL_UNITS_PER_ROTATION/ Constants.Autonomous.PigeonPID.PIGEON_UNITS_PER_ROTATION, 1, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        //Config output & sensor direction
        Robot.RIGHT_SIDE.setSensorPhase(true); // Must be set before setInverted()
        Robot.LEFT_SIDE.setSensorPhase(true);
        Robot.RIGHT_SIDE.setInverted(false); // TODO Change if necessary
        Robot.LEFT_SIDE.setInverted(false);

        Robot.RIGHT_SIDE.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.PIGEON.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,5, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        Robot.RIGHT_SIDE.configNeutralDeadband(0.001, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.LEFT_SIDE.configNeutralDeadband(0.001, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        //Config peak outputs
        Robot.RIGHT_SIDE.configPeakOutputForward(+1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.LEFT_SIDE.configPeakOutputForward(+1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.configPeakOutputReverse(-1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.LEFT_SIDE.configPeakOutputReverse(-1.0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);

        //Set turn constants
        Robot.RIGHT_SIDE.config_kP(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, Constants.Autonomous.PigeonPID.TURNING_GAINS[0], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.config_kI(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, Constants.Autonomous.PigeonPID.TURNING_GAINS[1], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.config_kD(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, Constants.Autonomous.PigeonPID.TURNING_GAINS[2], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.config_kF(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, baseSpeed, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS);
        Robot.RIGHT_SIDE.config_IntegralZone(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, (int) Constants.Autonomous.PigeonPID.TURNING_GAINS[3], Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS); //
        Robot.RIGHT_SIDE.configAllowableClosedloopError(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, 0, Constants.Autonomous.PigeonPID.CONFIG_TIMEOUT_MS); // 0 Is an ID

    }

    @Override
    protected void execute()
    {
        Robot.RIGHT_SIDE.selectProfileSlot(Constants.Autonomous.PigeonPID.PID_TURNING_SLOT, 1);
        Robot.RIGHT_SIDE.set(ControlMode.PercentOutput, Constants.Autonomous.PigeonPID.TURNING_GAINS[4], DemandType.AuxPID, targetAngle);
        Robot.LEFT_SIDE.set(ControlMode.PercentOutput, -Robot.RIGHT_SIDE.getMotorOutputPercent());
        currentAngle = Robot.PIGEON.getAbsoluteCompassHeading(); // getAbsoluteCompassHeading returns a double 0-360
    }

    @Override
    protected void end()
    {
        Robot.DRIVE_TRAIN.runMotorsVelocity(0,0);
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
