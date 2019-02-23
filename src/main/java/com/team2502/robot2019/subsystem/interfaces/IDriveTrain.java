package com.team2502.robot2019.subsystem.interfaces;

import com.github.ezauton.core.action.require.IResource;
import com.github.ezauton.core.actuators.IVelocityMotor;
import com.github.ezauton.core.localization.IRotationalLocationEstimator;
import com.github.ezauton.core.localization.ITankRobotVelocityEstimator;
import com.github.ezauton.core.localization.ITranslationalLocationEstimator;
import com.github.ezauton.core.localization.Updateable;
import com.github.ezauton.core.localization.estimators.TankRobotEncoderEncoderEstimator;
import com.github.ezauton.core.localization.sensors.IVelocityEstimator;
import com.github.ezauton.core.robot.subsystems.TranslationalLocationDriveable;

/**
 * For the purposes of ezAuton + simulation
 */
public interface IDriveTrain extends TranslationalLocationDriveable, IResource, Updateable
{
    default void runMotorsVelocity(double leftVel, double rightVel) {
        IVelocityMotor left = getLeft();
        IVelocityMotor right = getRight();

        left.runVelocity(leftVel);
        right.runVelocity(rightVel);
    }

    void runMotorsVoltage(double leftVolts, double rightVolts);

    IVelocityMotor getLeft();

    IVelocityMotor getRight();

    ITranslationalLocationEstimator getLocEstimator();

    IRotationalLocationEstimator getRotEstimator();

    IVelocityEstimator getVelocityEstimator();

    IResource getResource();

    @Override
    default void take() throws InterruptedException
    {
        getResource().take();
    }

    @Override
    default void giveBack() {
        getResource().giveBack();
    }

    @Override
    default boolean isTakenByAnyone() { return getResource().isTakenByAnyone(); }

    @Override
    default void assertPossession() throws IllegalStateException {
        getResource().assertPossession();
    }
}
