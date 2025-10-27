package org.firstinspires.ftc.teamcode.control;

// all of teamcode

import org.firstinspires.ftc.teamcode.stuff.Robot;

// all (most) of java


/**
 * PID Controller
 */

public class PID {

    public double K_P;
    public double K_I;
    public double K_D;

    // high-pass filter
    public double hpf;

    public double e_PREVIOUS = 0D;

    private double e_INT = 0D;

    // PID Controller
    public PID(double K_P, double K_I, double K_D, double hpf) {
        this.K_P = K_P;
        this.K_I = K_I;
        this.K_D = K_D;
        this.hpf = hpf;
    }

    // set point = r(t)
    // measured = x(t)
    public double control(double r, double x) {
        // small change in time
        double dt = Robot.time - Robot.time_previous;

        // error = e(t)
        double e = r - x;

        // high-pass filter
        if (Math.abs(e) < hpf) {
            return 0;
        }

        // the derivative of error with respect to time (approximated)
        double e_DOT = (e - e_PREVIOUS) / dt;

        // the integral of error with respect to time (approximated)
        e_INT = e_INT + (e * dt);

        double u = K_P * (e) + K_I * (e_INT) + K_D * (e_DOT);

        e_PREVIOUS = e;

        return u;
    }
}
