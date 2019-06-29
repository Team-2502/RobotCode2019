package com.team2502.robot2019;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * A class with mostly static members to manage putting things onto the dashboard
 */
public final class DashboardData
{

    /**
     * A list of subsystems (which extend the functional interface DashboardUpdater)
     */
    private static List<DashboardUpdater> updaters = new ArrayList<>(4);

    private DashboardData() { }

    /**
     * Run each updater to update the dashboard
     */
    static void update()
    {
        Shuffleboard.update();
        for(DashboardUpdater subsystem : updaters) { subsystem.updateDashboard(); }
        updateVisionData();
//        ppRecord();
    }

    /**
     * Add an updater
     *
     * @param subsystem An updater (which is usually a subsystem) that will continually put stuff on the smartdashboard
     */
    public static void addUpdater(DashboardUpdater subsystem) { updaters.add(subsystem); }

    private static void updateVisionData()
    {
        //Robot.seesTarget.setBoolean(! (Robot.tvecs2Entry.getDouble(-9001) == -9001 || Robot.angleEntry.getDouble(-9001) == -9001));
    }


    /**
     * An interface to allow you to automatically update stuff on the Smart Dashboard.
     */
    @FunctionalInterface
    public interface DashboardUpdater
    {
        /**
         * Called every tick to update data on the Smart Dashboard.
         */
        void updateDashboard();
    }
}