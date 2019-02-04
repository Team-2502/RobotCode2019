package com.team2502.robot2019.subsystem.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;

public class VisionData
{
    private final ImmutableVector pos;
    private final double angle;


    public VisionData(double x, double y, double angle)
    {
        this.pos = new ImmutableVector(x, y);
        this.angle = angle;
    }

    public ImmutableVector getPos()
    {
        return pos;
    }

    public double getAngle()
    {
        return angle;
    }

    public boolean isMeaningful()
    {
        return pos.equals(new ImmutableVector(-9001, -9001)) && angle == -9001;
    }

    @Override
    public String toString()
    {
        final StringBuilder sb = new StringBuilder("VisionData{");
        sb.append("pos=").append(pos);
        sb.append(", angle=").append(angle);
        sb.append('}');
        return sb.toString();
    }
}