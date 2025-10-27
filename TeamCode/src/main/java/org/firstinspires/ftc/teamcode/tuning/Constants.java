package org.firstinspires.ftc.teamcode.tuning;

import org.firstinspires.ftc.teamcode.mathemagics.Point;

public class Constants {
    // PPR (pulses per revolution) = CPR (counts per revolution)
    // approximately 537.7 (source: gobilda website)
    public static final double CPR = ((((1D+(46D/17D))) * (1D+(46D/11D))) * 28D);

    public static final Point aprilTag1Pos = new Point(5,5);
    public static final double aprilTag1Heading = 0;

    public static final double OP_CPR = 2000;

    // mm to m
    public static final double WHEEL_DIAMETER = 96D / 1000;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double OP_DIAMETER = 48D / 1000;
    public static final double OP_CIRCUMFERENCE = OP_DIAMETER * Math.PI;

    // inches to m
    public static final double WIDTH = .428;
    public static final double LENGTH = .39878;

    // 24 inches converted to mm to meters...
    public static final double TILE_LENGTH = 24 * 2.54 / 100;

    // width of robot: 41.8 cm
    // length: 43.1 cm

    // CoR: 24.9, 16.55

    // horizontal odometry pod x: 28 cm
    // horizontal odometry pod y: 34.6 cm

    // vertical odometry pod x: 4 cm
    // vertical odometry pod y: 17 cm

    // center of rotation to odometry pod
//    public static final double CR_TO_X_POD = Math.sqrt(Math.pow(28 - 16.55, 2) + Math.pow(34.6 - 24.9, 2)) / 100;
    public static final double CR_TO_X_POD = Math.sqrt(Math.pow(.038,2) + Math.pow(.0875,2));

//    public static final double CR_TO_Y_POD = Math.sqrt(Math.pow(4 - 16.55, 2) + Math.pow(17 - 24.9, 2)) / 100;
    public static final double CR_TO_Y_POD = Math.sqrt(Math.pow(.053,2) + Math.pow(.044,2));

    // 0 = open
    // 1 = closed (has "obstacle")
    // 2 = closed diagonally (cannot traverse to node "diagonally")
    public static final int[][] RED_AUTONOMOUS_FIELD = {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 0
            {1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1}, // 1
            {1, 1, 1, 1, 1, 1, 1, 0, 2, 0, 2, 0, 1}, // 2
            {1, 1, 1, 1, 1, 1, 1, 0, 2, 2, 0, 0, 1}, // 3
            {1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 0, 0, 1}, // 4
            {1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 2, 0, 1}, // 5
            {1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1}, // 6
            {1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1}, // 7
            {1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 1}, // 8
            {1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 2, 0, 1}, // 9
            {1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1}, // 10
            {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1}, // 11
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 12
    };

    public static final int[][] BLUE_AUTONOMOUS_FIELD = {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 0
            {1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1}, // 1
            {1, 0, 2, 0, 2, 0, 1, 1, 1, 1, 1, 1, 1}, // 2
            {1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1}, // 3
            {1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1}, // 4
            {1, 0, 2, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1}, // 5
            {1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1}, // 6
            {1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1}, // 7
            {1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1}, // 8
            {1, 0, 2, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1}, // 9
            {1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1}, // 10
            {1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1}, // 11
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 12
    };


    // 0 = open
    // 1 = closed (has "obstacle")
    // 2 = closed diagonally (cannot traverse to node "diagonally")
    public static final int[][] TELEOP_FIELD = {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 0
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, // 1
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, // 2
            {1, 0, 0, 2, 2, 0, 0, 0, 2, 2, 0, 0, 1}, // 3
            {1, 0, 0, 2, 1, 0, 0, 0, 1, 2, 0, 0, 1}, // 4
            {1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1}, // 5
            {1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1}, // 6
            {1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1}, // 7
            {1, 0, 0, 2, 1, 0, 0, 0, 1, 2, 0, 0, 1}, // 8
            {1, 0, 0, 2, 2, 0, 0, 0, 2, 2, 0, 0, 1}, // 9
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, // 10
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, // 11
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 12
    };

    // oh hell naw, i ain't got the time for this :skull:
    public static final double[] FACTORIAL = {
            1,
            1,
            2,
            6,
            24,
            120,
            720,
            5040,
            40320,
            362880,
            3628800,
            39916800,
            479001600,
            479001600 * 13D,
            479001600 * 13D * 14D,
            479001600 * 13D * 14D * 15D,
            479001600 * 13D * 14D * 15D * 16D
    };

    public static final double[][] Q = {{1, 0}, {0, 1}};
    public static final double[][] R = {{0.01, 0}, {0, 0.01}};
}
