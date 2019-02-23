import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.PurePursuitAction;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.LookaheadBounds;
import com.github.ezauton.core.pathplanning.purepursuit.PPWaypoint;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.simulation.TimeWarpedSimulation;
import com.github.ezauton.recorder.Recording;
import com.github.ezauton.recorder.base.PurePursuitRecorder;
import com.github.ezauton.recorder.base.RobotStateRecorder;
import com.github.ezauton.recorder.base.TankDriveableRecorder;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.subsystem.sim.SimulatedDrivetrain;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class PathTest
{
    private LookaheadBounds lookahead;
    private TimeWarpedSimulation simulation;
    private SimulatedDrivetrain driveTrain;

    @BeforeEach
    public void init()
    {
        simulation = new TimeWarpedSimulation(10);
        driveTrain = new SimulatedDrivetrain(simulation.getClock());
        lookahead = Constants.Autonomous.getLookaheadBounds(driveTrain);

    }

    private class PPPair
    {
        private final PurePursuitAction command;
        private final PurePursuitRecorder purePursuitRecorder;

        public PPPair(PurePursuitAction command, PurePursuitRecorder purePursuitRecorder)
        {
            this.command = command;
            this.purePursuitRecorder = purePursuitRecorder;
        }

        public PurePursuitAction getCommand()
        {
            return command;
        }

        public PurePursuitRecorder getPurePursuitRecorder()
        {
            return purePursuitRecorder;
        }
    }

    private PPPair getActionAndRecorder(Path path)
    {
        PurePursuitMovementStrategy ppms = new PurePursuitMovementStrategy(path, 0.01);
        return new PPPair(
                new PurePursuitAction(20, TimeUnit.MILLISECONDS, ppms, driveTrain.getLocEstimator(), lookahead, driveTrain.getTankRobotTransLocDriveable()),
                new PurePursuitRecorder(simulation.getClock(), path, ppms)
        );
    }

    @Test
    public void testRightHab2Front() throws TimeoutException, ExecutionException
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

        testPath(paths, "testRightHab2Front");
    }

    @Test
    public void testRightHab2Rocket() throws TimeoutException, ExecutionException
    {
        List<Path> paths = Arrays.asList(
                new SplinePPWaypoint.Builder()
                        .add(0, 0, 0, 10, 13, -12)
                        .add(0, 9, 0, 10, 13, -13)
                        .add(6.5, 24.15, .34 * Math.PI, 0, 13, -12)
                        .buildPathGenerator()
                        .generate(0.05),
                new PPWaypoint.Builder()
                        .add(6.5, 24.15, -5, 13, -12)
                        .add(8.5, 20, 0, 13, -12)
                        .buildPathGenerator().generate(0.05),
                new PPWaypoint.Builder()
                        .add(8.5, 20, 10, 13, -12)
                        .add(6.5, 24.15, 0, 13, -12)
                        .buildPathGenerator().generate(0.05),
                new SplinePPWaypoint.Builder()
                        .add(6.5, 24.15, Math.PI, -15, 13, -12)
                        .add(7.75, 0, Math.PI, -1, 13, -12)
                        .buildPathGenerator().generate(0.05),
                new SplinePPWaypoint.Builder()
                        .add(7.75, 0, 0, 15, 13, -12)
                        .add(3.75, 10, Math.PI / 2, 1, 13, -12)
                        .buildPathGenerator().generate(0.05),
                new SplinePPWaypoint.Builder()
                        .add(3.75, 10, 3 * Math.PI / 2, -10, 13, -12)
                        .add(8.5, 17, 1.66 * Math.PI, -15, 13, -12)
                        .buildPathGenerator().generate(0.05)
                                        );

        testPath(paths, "testRightHab2Rocket");
    }

    private void testPath(List<Path> paths, String name) throws TimeoutException, ExecutionException
    {
        Recording recording = new Recording();
        ActionGroup group = new ActionGroup();
        ActionGroup PurePursuitActions = new ActionGroup();


        paths.stream().map(this::getActionAndRecorder).forEach((PPPair p) -> {
            PurePursuitActions.addSequential(p.getCommand());
            recording.addSubRecording(p.getPurePursuitRecorder());
        });

        RobotStateRecorder posRec = new RobotStateRecorder("robotstate", simulation.getClock(), driveTrain.getLocEstimator(), driveTrain.getLocEstimator(), Constants.Physical.DriveTrain.LATERAL_WHEEL_DIST_FT, 2);
        TankDriveableRecorder tankRobot = new TankDriveableRecorder("td", simulation.getClock(), driveTrain.getTankRobotTransLocDriveable());

        recording.addSubRecording(posRec);
        recording.addSubRecording(tankRobot);


        group.with(new BackgroundAction(1, TimeUnit.MILLISECONDS, driveTrain::update))
             .with(new BackgroundAction(7, TimeUnit.MILLISECONDS, recording::update))
             .addSequential(PurePursuitActions);


        try
        {
            simulation.add(group)
                      .runSimulation(45, TimeUnit.SECONDS);
        }
        catch(TimeoutException | ExecutionException e)
        {
            e.printStackTrace();
        }
        try
        {
            recording.save(name + ".json");
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
    }
}
