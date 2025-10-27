package org.firstinspires.ftc.teamcode.mathemagics;

import static org.firstinspires.ftc.teamcode.mathemagics.Matrix.add;
import static org.firstinspires.ftc.teamcode.mathemagics.Matrix.inverse;
import static org.firstinspires.ftc.teamcode.mathemagics.Matrix.multiply;
import static org.firstinspires.ftc.teamcode.mathemagics.Matrix.subtract;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Stack;

/**
 * orz
 */

public class Oikura {

    // discrete-time algebraic Riccati equation (DARE)
    /*
        P_{i - 1} = (A^{T} P_{i} A) – (A^{T} P_{i} B) (R + B^{T} P_{i} B)^{-1} (B^{T} P_{i} A) + Q
    */

    public static double[][] DARE(double[][] A, double[][] B, double[][] Q, double[][] R, double[][] P) {
        // disassemble the equation into parts

        double[][] a = multiply(multiply(Matrix.transpose(A), P), A);
        double[][] b = multiply(multiply(Matrix.transpose(A), P), B);
        double[][] c = inverse(add(R, multiply(multiply(Matrix.transpose(B), P), B)));
        double[][] d = multiply(multiply(Matrix.transpose(B), P), A);

        return add(subtract(a, multiply(multiply(b, c), d)), Q);
    }

    // discrete-time optimal control feedback gains
    /*
        K_i = (R + B^{T} P_{i + 1} B)^{-1} (B^{T} P_{i + 1} A)
    */

    public static double[][] K(double[][] A, double[][] B, double[][] R, double[][] P) {
        // disassemble the equation into parts

        double[][] a = inverse(add(R, multiply(multiply(Matrix.transpose(B), P), B)));
        double[][] b = multiply(multiply(Matrix.transpose(B), P), A);

        return multiply(a, b);
    }

    // optimal control/input
    /*
        u = -K e
    */

    public static double[][] u(double[][] K, double[][] e) {
        return multiply(multiply(-1, K), e);
    }

    public static Queue<BezierCurve> trace(Stack<Point> nodes) {
        Queue<BezierCurve> path = new LinkedList<>();

        double size = nodes.size();

        double dy_1 = nodes.get(nodes.size() - 2).y - nodes.peek().y;
        double dx_1 = nodes.get(nodes.size() - 2).x - nodes.peek().x;

        double dy_2;
        double dx_2;

        for (int i = 0; i < size - 1; i = i + 1) {
            Point begin = nodes.pop();
            Point end = nodes.peek();

            double distance = begin.distanceTo(end);

            dy_2 = nodes.get(Math.max(0, nodes.size() - 2)).y - begin.y;
            dx_2 = nodes.get(Math.max(0, nodes.size() - 2)).x - begin.x;

            // control points to push to path
            Point p_left = new Point(
                    begin.x + distance / 3 * Math.cos(Math.atan2(dy_1, dx_1)),
                    begin.y + distance / 3 * Math.sin(Math.atan2(dy_1, dx_1))
            );
            Point p_right = new Point(
                    end.x - distance / 3 * Math.cos(Math.atan2(dy_2, dx_2)),
                    end.y - distance / 3 * Math.sin(Math.atan2(dy_2, dx_2))
            );

            BezierCurve curve = new BezierCurve();

            curve.add_point(begin);
            curve.add_point(p_left);
            curve.add_point(p_right);
            curve.add_point(end);

            path.add(curve);

            dy_1 = dy_2;
            dx_1 = dx_2;
        }

        return path;
    }
}
