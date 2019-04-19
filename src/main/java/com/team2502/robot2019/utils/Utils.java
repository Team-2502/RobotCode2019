package com.team2502.robot2019.utils;

import com.team2502.robot2019.Constants;

public class Utils
{
    public static double handleMaxAcc(double desiredVelocity, double lastVelocity, double dt, double maxAcc) {

        double currentAcc = Math.abs(desiredVelocity - lastVelocity) / dt;
        double ret;
        if(currentAcc < maxAcc) { // within bounds
            ret = desiredVelocity; // as is
        }
        else // out of bounds
        {
            // Default acceleration would be too high; limit setpoint to respect maximum acceleration
            double sign = Math.signum(desiredVelocity - lastVelocity); // limit
            ret = lastVelocity + sign * Constants.Physical.DriveTrain.MAX_FPS2_ACCEL * dt;
        }
        System.out.println(currentAcc);
        return ret;
    }

    public static double furthestFromZero(double a, double b) {
        if(Math.max(Math.abs(a), Math.abs(b)) == Math.abs(a)) {
            return a;
        }
        return b;
    }
}
