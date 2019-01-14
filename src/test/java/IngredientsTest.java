import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.PPCommand;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.LookaheadBounds;
import com.github.ezauton.core.pathplanning.purepursuit.PPWaypoint;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.simulation.TimeWarpedSimulation;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.command.autonomous.ingredients.VoltageDriveAction;
import com.team2502.robot2019.subsystem.sim.SimulatedDrivetrain;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class IngredientsTest
{
    private SimulatedDrivetrain driveTrain;
    private TimeWarpedSimulation sim;

    @Before
    public void init() {
        sim = new TimeWarpedSimulation();
        driveTrain = new SimulatedDrivetrain(sim.getClock());
    }

    @Test
    public void testVoltageDrive() {
        sim.add(new VoltageDriveAction(1, 1, 1, TimeUnit.SECONDS, true, sim.getClock(), driveTrain));
        sim.add(new BackgroundAction(10, TimeUnit.MILLISECONDS, driveTrain::update));
        sim.runSimulation(10, TimeUnit.SECONDS);

        ImmutableVector lastLocation = driveTrain.getLocEstimator().estimateLocation();
        System.out.println("driveTrain.getLocEstimator().estimateLocation() = " + lastLocation);
        Assert.assertNotEquals("The simulated robot did not move", new ImmutableVector(0, 0), lastLocation);
    }

    @Test
    public void testPurePursuit() {
        LookaheadBounds lookaheadBounds = Constants.Autonomous.getLookaheadBounds(driveTrain);
        Path a = new PPWaypoint.Builder()
                .add(0, 0, 10, 5, -100)
                .add(8, 9, 10, 3, -100)
                .add(0, 20, 0, 5, -100)
                             .buildPathGenerator().generate(0.05);
        PurePursuitMovementStrategy ppstrat = new PurePursuitMovementStrategy(a, 0.000001);
        sim.add(new PPCommand(10, TimeUnit.MILLISECONDS, ppstrat, driveTrain.getLocEstimator(), lookaheadBounds, driveTrain));
        sim.add(new BackgroundAction(10, TimeUnit.MILLISECONDS, driveTrain::update));
        sim.runSimulation(10, TimeUnit.SECONDS);

        ImmutableVector lastLocation = driveTrain.getLocEstimator().estimateLocation();
        System.out.println("driveTrain.getLocEstimator().estimateLocation() = " + lastLocation);


    }
}
