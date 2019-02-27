package com.team2502.robot2019.subsystem.sim;

import com.github.ezauton.core.action.require.BaseResource;
import com.github.ezauton.core.actuators.VelocityMotor;
import com.github.ezauton.core.actuators.implementations.SimulatedMotor;
import com.github.ezauton.core.localization.RotationalLocationEstimator;
import com.github.ezauton.core.localization.UpdateableGroup;
import com.github.ezauton.core.localization.estimators.TankRobotEncoderEncoderEstimator;
import com.github.ezauton.core.localization.sensors.Encoders;
import com.github.ezauton.core.localization.sensors.TranslationalDistanceSensor;
import com.github.ezauton.core.localization.sensors.VelocityEstimator;
import com.github.ezauton.core.robot.implemented.TankRobotTransLocDriveable;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.Clock;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;


//TODO: Implement
public class SimulatedDrivetrain implements DriveTrain
{

    private final SimulatedMotor left;
    private final SimulatedMotor right;

    private final BaseResource resource = new BaseResource();
    private final TankRobotEncoderEncoderEstimator locEst;
    private final TranslationalDistanceSensor leftTds;
    private final TranslationalDistanceSensor rightTds;

    private final TankRobotTransLocDriveable tankRobotTransLocDriveable;

    private final UpdateableGroup updateableGroup;

    public SimulatedDrivetrain(Clock clock)
    {
        left = new SimulatedMotor(clock, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, 0.05, Constants.Physical.DriveTrain.MAX_FPS_SPEED, 12 / Constants.Physical.DriveTrain.MAX_FPS_SPEED);
        right = new SimulatedMotor(clock, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, 0.05, Constants.Physical.DriveTrain.MAX_FPS_SPEED, 12 / Constants.Physical.DriveTrain.MAX_FPS_SPEED);

        leftTds = Encoders.toTranslationalDistanceSensor(1, 1, left);
        rightTds = Encoders.toTranslationalDistanceSensor(1, 1, right);

        locEst = new TankRobotEncoderEncoderEstimator(leftTds, rightTds, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);
        tankRobotTransLocDriveable = new TankRobotTransLocDriveable(left, right, locEst, locEst, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);

        locEst.reset();
        updateableGroup = new UpdateableGroup(left, right, locEst);
    }

    @Override
    public void runMotorsVoltage(double leftVolts, double rightVolts)
    {
        left.runVoltage(leftVolts);
        right.runVoltage(rightVolts);
    }

    @Override
    public VelocityMotor getLeft()
    {
        return left;
    }

    @Override
    public VelocityMotor getRight()
    {
        return right;
    }

    @Override
    public TankRobotEncoderEncoderEstimator getLocEstimator()
    {
        return locEst;
    }

    @Override
    public RotationalLocationEstimator getRotEstimator()
    {
        return locEst;
    }

    @Override
    public VelocityEstimator getVelocityEstimator()
    {
        return locEst;
    }

    @Override
    public boolean driveTowardTransLoc(double v, ImmutableVector immutableVector)
    {
        return tankRobotTransLocDriveable.driveTowardTransLoc(v, immutableVector);
    }

    @Override
    public boolean driveSpeed(double v)
    {
        return tankRobotTransLocDriveable.driveSpeed(v);
    }

    @Override
    public BaseResource getResource()
    {
        return resource;
    }

    @Override
    public boolean update()
    {
        return updateableGroup.update();
    }

    public TankRobotTransLocDriveable getTankRobotTransLocDriveable()
    {
        return tankRobotTransLocDriveable;
    }
}
