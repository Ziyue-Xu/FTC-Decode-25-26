package org.firstinspires.ftc.teamcode.mathemagics;

/**
 * A point in a coordinate system.
 */

public class Point {
    // rectangular coordinate system
    public double x;
    public double y;

    // polar coordinate system (r, theta in radians)
    public double r;
    public double theta;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;

        r = Math.sqrt(x * x + y * y);
        theta = Math.atan2(y, x);
    }

    public Point(Point a, double distX, double distY) {
        x = a.x-distX;
        y = a.y-distY;

        r = Math.sqrt(x * x + y * y);
        theta = Math.atan2(y, x);
    }

    public double distanceTo(Point point) {
        return Math.sqrt(Math.pow(x - point.x, 2) + Math.pow(y - point.y, 2));
    }

    public double distanceTo(double x, double y) {
        return Math.sqrt(Math.pow(this.x - x, 2) + Math.pow(this.y - y, 2));
    }

    public double slopeTo(Point point) {
        if (x - point.x != 0) {
            return ((y - point.y) / (x - point.x));
        } else {
            return 69420;
        }
    }

    @Override
    public boolean equals(Object point) {
        return (x == ((Point) point).x) && (y == ((Point) point).y);
    }
}
