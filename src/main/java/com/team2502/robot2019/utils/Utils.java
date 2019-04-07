package com.team2502.robot2019.utils;

import com.team2502.robot2019.Constants;

public class Utils
{
    public static double handleMaxAcc(double desiredVelocity, double lastVelocity, double dt) {

        double currentAcc = Math.abs(desiredVelocity - lastVelocity) / dt;
        double ret;
        if(currentAcc < Constants.Physical.DriveTrain.MAX_FPS2_ACCEL) { // within bounds
            ret = desiredVelocity; // as is
        }
        else // out of bounds
        {
            System.out.println("ur driving 2 fast");
            double sign = Math.signum(desiredVelocity - lastVelocity); // limit
            ret = lastVelocity + sign * Constants.Physical.DriveTrain.MAX_FPS2_ACCEL * dt;
        }
        System.out.println(currentAcc);
        return ret;
    }
}
