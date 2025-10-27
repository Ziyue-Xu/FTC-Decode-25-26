package org.firstinspires.ftc.teamcode.mathemagics;

// all (most) of java

public class Angular {

    /**
     * Returns smallest/optimal angular displacement between two angles
     * @param begin initial angle
     * @param end final angle
     * @return angular displacement
     */
    public static double optimal_angular_displacement(double begin, double end) {
        // positive is counterclockwise, negative is clockwise
        double angular_displacement = end - begin;

        // two angles are "equivalent" when:
        // angle A = angle B ± n * 2 * pi

        if (angular_displacement < -Math.PI) {
            angular_displacement = angular_displacement + 2 * Math.PI;
        } else if (angular_displacement > Math.PI) {
            angular_displacement = angular_displacement - 2 * Math.PI;
        }

        return angular_displacement;
    }
}
