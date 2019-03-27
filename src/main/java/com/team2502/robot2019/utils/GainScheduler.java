package com.team2502.robot2019.utils;


import edu.wpi.first.wpilibj.PIDBase;

import java.util.Iterator;
import java.util.Objects;
import java.util.SortedMap;
import java.util.TreeMap;

public class GainScheduler
{
    private final SortedMap<Double, PIDTriple> map = new TreeMap<>();

    public GainScheduler(double defaultKP, double defaultKI, double defaultKD)
    {
        map.put(Double.POSITIVE_INFINITY, new PIDTriple(defaultKP, defaultKI, defaultKD));
    }


    /**
     * Schedule separate gains for a different error bound
     *
     * @param maxError The maximum error for which this gain should apply
     * @param kP
     * @param kI
     * @param kD
     * @return this
     */
    public GainScheduler scheduleGains(double maxError, double kP, double kI, double kD)
    {
        if(maxError < 0)
        {
            throw new RuntimeException("maxError cannot be negative");
        }

        map.put(maxError, new PIDTriple(kP, kI, kD));
        return this;
    }

    public SortedMap<Double, PIDTriple> getMap()
    {
        return map;
    }

    public void applyScheduledGains(PIDBase pidController)
    {
        PIDTriple pidToUse = applyScheduledGains(pidController.getError());
        pidController.setPID(pidToUse.kP, pidToUse.kI, pidToUse.kD);
    }

    public PIDTriple applyScheduledGains(double error) {
        Iterator<Double> gainMaxErrorVals = map.keySet().iterator();
        double maxErrorVal = gainMaxErrorVals.next();
        while(error > maxErrorVal) {
            maxErrorVal = gainMaxErrorVals.next();
        }

        // At this point, error <= maxErrorVal
        return map.get(maxErrorVal);
    }

    public static class PIDTriple
    {
        private final double kP;
        private final double kI;
        private final double kD;

        public PIDTriple(double kP, double kI, double kD)
        {

            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double getkP()
        {
            return kP;
        }

        public double getkI()
        {
            return kI;
        }

        public double getkD()
        {
            return kD;
        }

        @Override
        public String toString()
        {
            final StringBuilder sb = new StringBuilder("PIDTriple{");
            sb.append("kP=").append(kP);
            sb.append(", kI=").append(kI);
            sb.append(", kD=").append(kD);
            sb.append('}');
            return sb.toString();
        }

        @Override
        public boolean equals(Object o)
        {
            if(this == o) { return true; }
            if(o == null || getClass() != o.getClass()) { return false; }
            PIDTriple pidTriple = (PIDTriple) o;
            return Double.compare(pidTriple.kP, kP) == 0 &&
                   Double.compare(pidTriple.kI, kI) == 0 &&
                   Double.compare(pidTriple.kD, kD) == 0;
        }

        @Override
        public int hashCode()
        {
            return Objects.hash(kP, kI, kD);
        }
    }
}
