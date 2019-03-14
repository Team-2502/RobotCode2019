package com.team2502.robot2019.utils;

import com.team2502.robot2019.DashboardData;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoringHUD implements DashboardData.DashboardUpdater
{
    private final boolean[] leftHatch;
    private final boolean[] rightHatch;
    private final boolean[] leftCargo;
    private final boolean[] rightCargo;

    public ScoringHUD()
    {
        leftHatch = new boolean[3];
        rightHatch = new boolean[3];
        leftCargo = new boolean[3];
        rightCargo = new boolean[3];
        DashboardData.addUpdater(this);
    }

    public void increment(int zone, boolean left)
    {
        if (left)
        {
            if (!leftHatch[zone])
                leftHatch[zone] = true;
            else if (!leftCargo[zone])
                leftCargo[zone] = true;
            else
            {
                leftHatch[zone] = false;
                leftHatch[zone] = false;
            }
        }

        else
        {
            if (!rightHatch[zone])
                rightHatch[zone] = true;
            else if (!rightCargo[zone])
                rightCargo[zone] = true;
            else
            {
                rightHatch[zone] = false;
                rightHatch[zone] = false;
            }
        }
    }


    @Override
    public void updateDashboard()
    {
        for(int i = 0; i < 2; i++) {
            SmartDashboard.putBoolean(String.format("leftCargo %d", i), leftCargo[i]);
            SmartDashboard.putBoolean(String.format("rightCargo %d", i), rightCargo[i]);
            SmartDashboard.putBoolean(String.format("leftHatch %d", i), leftHatch[i]);
            SmartDashboard.putBoolean(String.format("rightHatch %d", i), rightHatch[i]);
        }
    }
}
