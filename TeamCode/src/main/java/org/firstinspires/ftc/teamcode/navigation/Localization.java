package org.firstinspires.ftc.teamcode.navigation;

import static org.firstinspires.ftc.teamcode.stuff.Robot.X;
import static org.firstinspires.ftc.teamcode.stuff.Robot.Y;
import static org.firstinspires.ftc.teamcode.stuff.Robot.imu;
import static org.firstinspires.ftc.teamcode.stuff.Robot.initial_theta;
import static org.firstinspires.ftc.teamcode.stuff.Robot.theta;
import static org.firstinspires.ftc.teamcode.stuff.Robot.time;
import static org.firstinspires.ftc.teamcode.stuff.Robot.time_previous;
import static org.firstinspires.ftc.teamcode.tuning.Constants.CPR;
import static org.firstinspires.ftc.teamcode.tuning.Constants.CR_TO_X_POD;
import static org.firstinspires.ftc.teamcode.tuning.Constants.CR_TO_Y_POD;
import static org.firstinspires.ftc.teamcode.tuning.Constants.OP_CIRCUMFERENCE;
import static org.firstinspires.ftc.teamcode.tuning.Constants.OP_CPR;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mathemagics.Angular;
import org.firstinspires.ftc.teamcode.stuff.Robot;
import org.firstinspires.ftc.teamcode.tuning.Constants;

public class Localization {

    // encoder counts for respective (mecanum) wheels
    public static double FL = 0;
    public static double BR = 0;
    public static double FR = 0;
    public static double BL = 0;

    // linear and angular kinematics quantities
    public static double ABSOLUTE_X_VELOCITY = 0;
    public static double ABSOLUTE_Y_VELOCITY = 0;

    public static double RELATIVE_X_VELOCITY = 0;
    public static double RELATIVE_Y_VELOCITY = 0;

    // encoder counts for respective dead wheel(s)/odometry pod(s)
    public static double X_POD = 0;
    public static double Y_POD = 0;

    // (mecanum) encoder localization (cool but inaccurate)
    public static void localize(double fl, double br, double fr, double bl) {
        theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + initial_theta;

        // tangential/linear velocities of mecanum wheels {fl, br, fr, bl}
        double dt = time - time_previous;

        double v_fl = Constants.WHEEL_CIRCUMFERENCE * ((fl - FL) / CPR) / dt;
        double v_br = Constants.WHEEL_CIRCUMFERENCE * ((br - BR) / CPR) / dt;
        double v_fr = Constants.WHEEL_CIRCUMFERENCE * ((fr - FR) / CPR) / dt;
        double v_bl = Constants.WHEEL_CIRCUMFERENCE * ((bl - BL) / CPR) / dt;

        // update encoder counts
        FL = fl;
        BR = br;

        FR = fr;
        BL = bl;

        double RELATIVE_Y_VELOCITY_X = Math.cos(theta) * RELATIVE_Y_VELOCITY;
        double RELATIVE_Y_VELOCITY_Y = Math.sin(theta) * RELATIVE_Y_VELOCITY;

        double RELATIVE_X_VELOCITY_X = Math.sin(theta) * RELATIVE_X_VELOCITY;
        double RELATIVE_X_VELOCITY_Y = -Math.cos(theta) * RELATIVE_X_VELOCITY;

        // Y

        // kinematics with previous ABSOLUTE_Y_VELOCITY
        Y = Y + (ABSOLUTE_Y_VELOCITY / 2) * dt;

        // update RELATIVE_Y_VELOCITY and ABSOLUTE_Y_VELOCITY
        // meters per second ;-;
        RELATIVE_Y_VELOCITY = (v_fl + v_br + v_fr + v_bl) / 4;
        ABSOLUTE_Y_VELOCITY = RELATIVE_Y_VELOCITY_Y + RELATIVE_X_VELOCITY_Y;

        // kinematics with current ABSOLUTE_Y_VELOCITY
        Y = Y + (ABSOLUTE_Y_VELOCITY / 2) * dt;

        // X
        // dx = (v_i + v_f) / 2 * dt

        // kinematics with previous ABSOLUTE_X_VELOCITY
        X = X + (ABSOLUTE_X_VELOCITY / 2) * dt;

        // update RELATIVE_X_VELOCITY and ABSOLUTE_X_VELOCITY
        // meters per second ;-;
        RELATIVE_X_VELOCITY = (v_fl + v_br - v_fr - v_bl) / 4;
        ABSOLUTE_X_VELOCITY = RELATIVE_Y_VELOCITY_X + RELATIVE_X_VELOCITY_X;

        // kinematics with current ABSOLUTE_X_VELOCITY
        X = X + (ABSOLUTE_X_VELOCITY / 2) * dt;
    }

    // dead wheel/odometry pod localization (less cool but accurate)
    public static void localize(double x_pod, double y_pod) {
        double previous_theta = theta;

        theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + initial_theta;

        double dx = (x_pod - X_POD) / OP_CPR * OP_CIRCUMFERENCE;
        System.out.println(x_pod-X_POD);
        System.out.println(dx + " = pre arclength subtract");
        dx = dx - Angular.optimal_angular_displacement(previous_theta, theta) * CR_TO_X_POD;
        System.out.println(dx + " = after arclength subtract");

        double dy = (y_pod - Y_POD) / OP_CPR * OP_CIRCUMFERENCE;
        System.out.println(dy + " = dy pre arclength subtract");
        dy = dy - Angular.optimal_angular_displacement(previous_theta, theta) * CR_TO_Y_POD;
        System.out.println(dy + " = dy after arclength subtract");
        Robot.X = Robot.X + Math.cos(theta) * dy + Math.sin(theta) * dx;
        System.out.println("delta x = " + (Math.cos(theta + Math.PI/2) * dy + Math.cos(theta) * dx));
        X_POD = x_pod;

        Robot.Y = Robot.Y + Math.sin(theta) * dy + Math.cos(theta) * dx;

        Y_POD = y_pod;
    }
}
