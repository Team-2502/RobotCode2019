package com.team2502.robot2019.utils;

import com.github.ezauton.core.action.*;
import com.github.ezauton.core.pathplanning.PP_PathGenerator;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.robot.subsystems.TranslationalLocationDriveable;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.Clock;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.recorder.Recording;
import com.github.ezauton.recorder.base.PurePursuitRecorder;
import com.github.ezauton.recorder.base.RobotStateRecorder;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;
import com.team2502.robot2019.subsystem.interfaces.HatchFlap;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

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
            public static final SplinePPWaypoint.Builder segment1 =
                    new SplinePPWaypoint.Builder()
                            .add(0, 0, 0, 4, 4, -4)
                            .add(0, 3, 0, 4, 4, -4)
                            .add(.95, 12, 0, 3.1, 4, -4);
//                            .add(.95, 14.5 - (30 / 12D), 0, 0, 13, -4)
//                            .buildPathGenerator();

            public static final SplinePPWaypoint.Builder segment2 =
                    new SplinePPWaypoint.Builder()
                            .add(.95, 14.5 - (30 / 12D), Math.PI, -10, 13, -4)
                            .add(10.9, 6.75, Math.PI, -7.5, 13, -4)
                            .add(11.3, 0, Math.PI, -5, 13, -4);
//                            .buildPathGenerator();

            public static final SplinePPWaypoint.Builder segment1Flipped = segment1.flipY();
            public static final SplinePPWaypoint.Builder segment2Flipped = segment2.flipY();

            public static ActionGroup getAction_CenterToRightCargoShip(DriveTrain dt, HatchFlap hf, Clock clock)
            {
                PPBean firstSegmentBean = generatePPAction(dt, segment1.buildPathGenerator());
                PPBean secondSegmentBean = generatePPAction(dt, segment2.buildPathGenerator());

                ActionGroup movementGroup = new ActionGroup();
                movementGroup.addSequential(firstSegmentBean.getPpAction());
//                movementGroup.addSequential(new BaseAction(Robot.HATCH_INTAKE::toggleHatchIntake));
                movementGroup.addSequential(secondSegmentBean.getPpAction());

                Recording rec = new Recording();
                ActionGroup group = new ActionGroup();
                rec.addSubRecording(new PurePursuitRecorder(clock, firstSegmentBean.ppms.getPath(), firstSegmentBean.getPpms()));
                rec.addSubRecording(new PurePursuitRecorder(clock, secondSegmentBean.ppms.getPath(), secondSegmentBean.getPpms()));
                rec.addSubRecording(new RobotStateRecorder(clock, dt.getLocEstimator(),dt.getRotEstimator(), Constants.Physical.DriveTrain.ROBOT_WIDTH_FT, Constants.Physical.DriveTrain.ROBOT_LENGTH_FT));

                group.with(new BackgroundAction(25, TimeUnit.MILLISECONDS, rec::update));
                group.with(new BackgroundAction(5, TimeUnit.MILLISECONDS, dt::update));
                group.addSequential(movementGroup);

                group.addSequential(() -> {
                    try
                    {
                        rec.save("apple.json");
                    }
                    catch(IOException e)
                    {
                        e.printStackTrace();
                    }
                });

                return group;
            }

            public static ActionGroup getAction_CenterToLeftCargoShip(DriveTrain dt, HatchFlap hf, Clock clock)
            {
                PPBean firstSegmentBean = generatePPAction(dt, segment1Flipped.buildPathGenerator());
                PPBean secondSegmentBean = generatePPAction(dt, segment2Flipped.buildPathGenerator());

                ActionGroup movementGroup = new ActionGroup();
                movementGroup.addSequential(firstSegmentBean.getPpAction());
//                movementGroup.addSequential(new BaseAction(Robot.HATCH_INTAKE::toggleHatchIntake));
                movementGroup.addSequential(secondSegmentBean.getPpAction());

                Recording rec = new Recording();
                ActionGroup group = new ActionGroup();
                rec.addSubRecording(new PurePursuitRecorder(clock, firstSegmentBean.ppms.getPath(), firstSegmentBean.getPpms()));
                rec.addSubRecording(new PurePursuitRecorder(clock, secondSegmentBean.ppms.getPath(), secondSegmentBean.getPpms()));
                rec.addSubRecording(new RobotStateRecorder(clock, dt.getLocEstimator(),dt.getRotEstimator(), Constants.Physical.DriveTrain.ROBOT_WIDTH_FT, Constants.Physical.DriveTrain.ROBOT_LENGTH_FT));

                group.with(new BackgroundAction(25, TimeUnit.MILLISECONDS, rec::update));
                group.with(new BackgroundAction(5, TimeUnit.MILLISECONDS, dt::update));
                group.addSequential(movementGroup);

                group.addSequential(() -> {
                    try
                    {
                        rec.save("apple.json");
                    }
                    catch(IOException e)
                    {
                        e.printStackTrace();
                    }
                });

                return group;
            }

        }
    }

    @NotNull
    private static PPBean generatePPAction(DriveTrain dt, PP_PathGenerator pathgen)
    {
        PurePursuitMovementStrategy ppms = new PurePursuitMovementStrategy(pathgen.generate(0.05), 1 / 12D);
        PurePursuitAction purePursuitAction = new PurePursuitAction(20,
                                                                    TimeUnit.MILLISECONDS,
                                                                    ppms,
                                                                    dt.getLocEstimator(),
                                                                    Constants.Autonomous.getLookaheadBounds(dt),
                                                                    new TranslationalLocationDriveable()
                                                                    {
                                                                        @Override
                                                                        public boolean driveTowardTransLoc(double speed, ImmutableVector loc)
                                                                        {
                                                                            return dt.driveTowardTransLoc(speed, loc);
                                                                        }

                                                                        @Override
                                                                        public boolean driveSpeed(double speed)
                                                                        {
                                                                            return dt.driveSpeed(speed);
                                                                        }
                                                                    });
        return new PPBean(purePursuitAction, ppms);
    }

    static class PPBean
    {
        private final PurePursuitAction ppAction;
        private final PurePursuitMovementStrategy ppms;

        public PPBean(PurePursuitAction ppAction, PurePursuitMovementStrategy ppms)
        {
            this.ppAction = ppAction;
            this.ppms = ppms;
        }

        public PurePursuitAction getPpAction()
        {
            return ppAction;
        }

        public PurePursuitMovementStrategy getPpms()
        {
            return ppms;
        }
    }
}
