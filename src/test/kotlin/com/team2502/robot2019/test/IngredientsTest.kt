package com.team2502.robot2019.test


import com.github.ezauton.core.action.*
import com.github.ezauton.core.pathplanning.purepursuit.PPWaypoint
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy
import com.github.ezauton.core.simulation.TimeWarpedSimulation
import com.github.ezauton.core.trajectory.geometry.ImmutableVector
import com.team2502.robot2019.Constants
import com.team2502.robot2019.command.autonomous.ingredients.VoltageDriveAction
import com.team2502.robot2019.subsystem.sim.SimulatedDrivetrain
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import java.util.concurrent.TimeUnit

class IngredientsTest {


    @Test
    fun testVoltageDrive() {

        val sim = TimeWarpedSimulation(10.0)
        val driveTrain = SimulatedDrivetrain(sim.clock)

        val actionGroup = ActionGroup()
                .with(BackgroundAction(10, TimeUnit.MILLISECONDS, Runnable { driveTrain.update() }))
                .addSequential(VoltageDriveAction(1.0, 1.0, 1.0, TimeUnit.SECONDS, true, sim.clock, driveTrain) as Action)

        sim.add(actionGroup).runSimulation(5, TimeUnit.SECONDS)

        val lastLocation = driveTrain.locEstimator.estimateLocation()
        println("driveTrain.getLocEstimator().estimateLocation() = $lastLocation")
        Assertions.assertNotEquals(ImmutableVector(0.0, 0.0), lastLocation, "The simulated robot did not move")
    }

    @Test
    fun testPurePursuit() {

        val sim = TimeWarpedSimulation(10.0)
        val driveTrain = SimulatedDrivetrain(sim.clock)

        val lookaheadBounds = Constants.Autonomous.getLookaheadBounds(driveTrain)
        val path = PPWaypoint.Builder()
                .add(0.0, 0.0, 10.0, 5.0, -5.0)
                .add(8.0, 9.0, 10.0, 3.0, -5.0)
                .add(0.0, 20.0, 0.0, 5.0, -5.0)
                .buildPathGenerator().generate(0.05)

        val tolerance = 0.1

        val ppstrat = PurePursuitMovementStrategy(path, tolerance)

        val group = ActionGroup()

        group
                .with(BackgroundAction(10, TimeUnit.MILLISECONDS, Runnable { driveTrain.update() }))
                .addSequential(PPCommand(10, TimeUnit.MILLISECONDS, ppstrat, driveTrain.locEstimator, lookaheadBounds, driveTrain))

        sim.add(group).runSimulation(10, TimeUnit.SECONDS)

        val lastLocation = driveTrain.locEstimator.estimateLocation()

        val final = path.pathSegments.last().to
        Assertions.assertTrue(lastLocation.dist(final) < tolerance*2, "Position: $lastLocation")
        //        System.out.println("driveTrain.getLocEstimator().estimateLocation() = " + lastLocation);


    }
}

