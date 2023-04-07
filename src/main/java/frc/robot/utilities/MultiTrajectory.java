// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.swervedrivespecialties.swervelib.control.Trajectory2;
import com.swervedrivespecialties.swervelib.control.Trajectory2.State;

/** Add your docs here. */
public class MultiTrajectory {
    double duration = 0;

    Trajectory2[] trajectories;
    double[] times;

    public MultiTrajectory(Trajectory2... trajectories) {
        times = new double[trajectories.length];
        int i = 0;
        for (Trajectory2 traj : trajectories) {
            duration += traj.getDuration();
            times[i] = duration;
            i++;
        }
        this.trajectories = trajectories;
    }

    public Trajectory2 getTrajectory(double time) {
        if(time>duration)
        {
            return trajectories[trajectories.length-1];
        }
        for (int i = 0; i < times.length; i++) {
            if (times[i] > time) {
                return trajectories[i];
            }
        }
        return null;
    }

    public int getIndex(double time) {
        if(time>duration)
        {
            return times.length-1;
        }
        for (int i = 0; i < times.length; i++) {
            if (times[i] > time) {
                return i;
            }
        }

        return -1;
    }

    public State calculate(double time) {
        if(trajectories.length==1)
        {
            return trajectories[0].calculate(time);
        }
        Trajectory2 traj = getTrajectory(time);
        if (getIndex(time) != 0) {
            return traj.calculate(time-times[getIndex(time)-1]);
        }
        else {
            return traj.calculate(time);
        }

    }

    public double getDuration() {
        return duration;
    }
}
