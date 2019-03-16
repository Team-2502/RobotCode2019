package com.team2502.robot2019.utils;

import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;

import java.util.Arrays;
import java.util.List;

public class Paths
{
    /**
     * Hand-made organic paths for starting on the right hand side of the HAB
     */
    public static class Right
    {
        public static class Level2
        {
            List<Path> paths = Arrays.asList(
                    new SplinePPWaypoint.Builder()
                            .add(0, 0, 0, 10, 13, -12)
                            .add(0, 9, 0, 10, 13, -13)
                            .add(3.5, 12.5, Math.PI / 2, 0, 13, -12)
                            .buildPathGenerator()
                            .generate(0.05),
                    new SplinePPWaypoint.Builder()
                            .add(3.5, 12.5, Math.PI / 2, -10, 13, -12)
                            .add(-2.562307, 16.241188, 0, -5, 13, -13)
                            .add(-2.562307, 18.241188, 0, 0, 13, -13)
                            .buildPathGenerator().generate(0.05),
                    new SplinePPWaypoint.Builder()
                            .add(-2.562307, 18.241188, Math.PI, 10, 13, -13)
                            .add(-2.562307, 16.241188, Math.PI, 10, 13, -13)
                            .add(3.5, 12.5, Math.PI / 2, 10, 13, -120)
                            .add(4, 14.7, 0, 0, 13, -12)
                            .buildPathGenerator().generate(0.05),
                    new SplinePPWaypoint.Builder()
                            .add(4, 14.7, -3 * Math.PI / 4, -10, 13, -12)
                            .add(7.75, 5, Math.PI, -10, 13, -12)
                            .add(7.75, 0, Math.PI, -2, 13, -12)
                            .buildPathGenerator().generate(0.05),
                    new SplinePPWaypoint.Builder()
                            .add(7.75, 0, 0, 10, 13, -12)
                            .add(7.75, 5, 0, 10, 13, -12)
                            .add(4, 14.7, 0, 10, 13, -12)
                            .buildPathGenerator().generate(0.05),
                    new SplinePPWaypoint.Builder()
                            .add(4, 14.7, 0, -10, 13, -12)
                            .add(3.5, 12.5, Math.PI / 2, -5, 13, -120)
                            .add(-4.562307, 16.241188, 0, -5, 13, -13)
                            .add(-4.562307, 18.241188, 0, 0, 13, -13)
                            .buildPathGenerator().generate(0.05)
                                            );
        }

        public static class Level1
        {

        }

    }
}
