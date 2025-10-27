package org.firstinspires.ftc.teamcode.navigation;

import static org.firstinspires.ftc.teamcode.tuning.Constants.TILE_LENGTH;

import org.firstinspires.ftc.teamcode.control.LQR;
import org.firstinspires.ftc.teamcode.mathemagics.Angular;
import org.firstinspires.ftc.teamcode.mathemagics.BezierCurve;
import org.firstinspires.ftc.teamcode.mathemagics.Oikura;
import org.firstinspires.ftc.teamcode.mathemagics.Point;
import org.firstinspires.ftc.teamcode.mechanics.Drivetrain;
import org.firstinspires.ftc.teamcode.stuff.Robot;

import java.util.Queue;

public class Motion {

    // high-pass filter
    public static double hpf = .01;

    public static Queue<BezierCurve> path = null;
    public static BezierCurve curve = null;

    public static int current;

    public static double t = 0;

    // A* + BezierCurve + LQR control
    // ...one of the greatest un-played moves in the history of FTC...
    public static boolean go_to(Point destination, int[][] FIELD) {
        // when initial position = final position...
        System.out.println("x dist = " + (destination.x - Robot.X));
        System.out.println("y dist = " + (destination.y- Robot.Y));
        System.out.println("hpf = " + hpf);
        if (Math.abs(destination.x - Robot.X) <= hpf && Math.abs(destination.y - Robot.Y) <= hpf) {
            System.out.println("it works?");
            return true;
        }

        if (path == null) {
            double[] x_i = {Math.floor(Robot.X / TILE_LENGTH * 2), Math.ceil(Robot.X / TILE_LENGTH * 2)};
            double[] y_i = {Math.floor(Robot.Y / TILE_LENGTH * 2), Math.ceil(Robot.Y / TILE_LENGTH * 2)};

            double[] x_f = {Math.floor(destination.x / TILE_LENGTH * 2), Math.ceil(destination.x / TILE_LENGTH * 2)};
            double[] y_f = {Math.floor(destination.y / TILE_LENGTH * 2), Math.ceil(destination.y / TILE_LENGTH * 2)};

            double x_i_optimal = -1;
            double y_i_optimal = -1;

            double x_f_optimal = -1;
            double y_f_optimal = -1;

            double size_optimal = 520;

            // i ran out of variables
            for (int __ = 0; __ <= 1; __ = __ + 1) {
                for (int ___ = 0; ___ <= 1; ___ = ___ + 1) {
                    for (int ____ = 0; ____ <= 1; ____ = ____ + 1) {
                        for (int _____ = 0; _____ <= 1; _____ = _____ + 1) {
                            A a = new A(new Field(FIELD));
                            System.out.println(__ + " is __ and ___ is " + ___);
                            System.out.println(Robot.X + " = robot.x and tile length = " + TILE_LENGTH);
                            if (a.field.isOpen((int) x_i[__], (int) y_i[___])) {
                                if (a.field.isOpen((int) x_f[____], (int) y_f[_____])) {
                                    double size = a.path(new Point(x_i[__], y_i[___]),
                                            new Point(x_f[____], y_f[_____]),
                                            new Point(Robot.X, Robot.Y),
                                            new Point(destination.x, destination.y)
                                    ).size();

                                    if (size_optimal > size) {
                                        size_optimal = size;

                                        x_i_optimal = x_i[__];
                                        y_i_optimal = y_i[___];
                                        x_f_optimal = x_f[____];
                                        y_f_optimal = y_f[_____];
                                    }
                                }
                            }
                        }
                    }
                }
            }

            A a = new A(new Field(FIELD));

            path = Oikura.trace(
                    a.path(
                            new Point(x_i_optimal, y_i_optimal),
                            new Point(x_f_optimal, y_f_optimal),
                            new Point(Robot.X, Robot.Y),
                            new Point(destination.x, destination.y)
                    )
            );

            current = path.size() - 1;

            curve = path.poll();

            t = 0;
        }

        if ((t >= 1) && (!path.isEmpty())) {
            curve = path.poll();

            t = 0;
        }

        t = Math.min(1, t + .285 / curve.distance());

        Point ideal_position = curve.B(t);

        drive_to(ideal_position);

        return Math.abs(destination.x - Robot.X) <= hpf && Math.abs(destination.y - Robot.Y) <= hpf;
    }

    // LQR control drive
    public static boolean drive_to(Point position) {
        double dt = Robot.time - Robot.time_previous;

        LQR lqr = new LQR(
                new double[][]{{1, 0}, {0, 1}},
                new double[][]{{dt, 0}, {0, dt}},
                new double[][]{{3, 0}, {0, 3}},
                new double[][]{{.005, 0}, {0, .005}}
        );

        double[][] powers = lqr.control(
                new double[][]{{Robot.X}, {Robot.Y}},
                new double[][]{{position.x}, {position.y}}
        );

        Drivetrain.x_power = Drivetrain.x_power + powers[0][0];
        Drivetrain.y_power = Drivetrain.y_power + powers[1][0];

        return Math.abs(position.x - Robot.X) <= hpf && Math.abs(position.y - Robot.Y) <= hpf;
    }

    public static boolean turn_to(double angle, double speed) {
        // note: -π <= IMU yaw readings <= π
        // so: -π/2 <= Robot.theta <= 3π/2

        // meaning we can only use theta values in the interval [-π/2, 3π/2].

        // -> which is 100% enough if we play our cards right.

        // reduce angle
        angle = angle % (2 * Math.PI);

        // find the optimal angular displacement between the current and desired angle
        angle = Angular.optimal_angular_displacement(Robot.theta, angle);

        double u = Robot.rotational_pid.control(angle, 0);

        // use PID controller to approach said desired angle (the angle will eventually reach 0)
        Drivetrain.rotational_power = Drivetrain.rotational_power - u * speed;

        return (u == 0);
    }

    public static double angle_to(Point position) {
        double i = position.x - Robot.X;
        double j = position.y - Robot.Y;

        // absolute angle = "field centric" angle
        double absolute_angle_to_location = Math.atan2(j, i);

        return absolute_angle_to_location;
    }
}
