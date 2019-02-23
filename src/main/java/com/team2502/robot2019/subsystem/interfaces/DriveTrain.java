package com.team2502.robot2019.subsystem.interfaces;

import com.github.ezauton.core.action.require.Resource;
import com.github.ezauton.core.actuators.VelocityMotor;
import com.github.ezauton.core.localization.RotationalLocationEstimator;
import com.github.ezauton.core.localization.TranslationalLocationEstimator;
import com.github.ezauton.core.localization.Updateable;
import com.github.ezauton.core.localization.sensors.VelocityEstimator;
import com.github.ezauton.core.robot.subsystems.TranslationalLocationDriveable;

/**
 * For the purposes of ezAuton + simulation
 */
public interface DriveTrain extends TranslationalLocationDriveable, Resource, Updateable
{
    default void runMotorsVelocity(double leftVel, double rightVel) {
        VelocityMotor left = getLeft();
        VelocityMotor right = getRight();

        left.runVelocity(leftVel);
        right.runVelocity(rightVel);
    }

    void runMotorsVoltage(double leftVolts, double rightVolts);

    VelocityMotor getLeft();

    VelocityMotor getRight();

    TranslationalLocationEstimator getLocEstimator();

    RotationalLocationEstimator getRotEstimator();

    VelocityEstimator getVelocityEstimator();

    Resource getResource();

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
