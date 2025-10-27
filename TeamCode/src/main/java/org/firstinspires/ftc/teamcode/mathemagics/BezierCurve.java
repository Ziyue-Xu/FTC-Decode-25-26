package org.firstinspires.ftc.teamcode.mathemagics;

import static org.firstinspires.ftc.teamcode.tuning.Constants.FACTORIAL;

import java.util.ArrayList;

public class BezierCurve {

    public ArrayList<Point> P;
    public int order = -1;

    public BezierCurve() {
        P = new ArrayList<>();
    }

    public void add_point(Point p) {
        P.add(p);

        order = order + 1;
    }

    public double X(double t) {
        return 0;
    }

    public double Y(double t) {
        return 0;
    }

    // fast but uncool
    public Point B(double t) {
        // B(t) = \sum_{i = 0}^{n} ({n \choose i} * (1 - t)^{n - i} * t^{i} * P_{i})

        Point B = new Point(0, 0);

        int N = P.size() - 1;

        for (int i = 0; i <= N; i = i + 1) {
            B.x = B.x + (FACTORIAL[N] / (FACTORIAL[i] * FACTORIAL[N - i])) * Math.pow(1 - t, N - i) * Math.pow(t, i) * P.get(i).x;
            B.y = B.y + (FACTORIAL[N] / (FACTORIAL[i] * FACTORIAL[N - i])) * Math.pow(1 - t, N - i) * Math.pow(t, i) * P.get(i).y;
        }

        return B;
    }

    // cool but slow
    public Point lerp(double t, int position, int order) {
        Point point;

        if (order == 1) {
            point = new Point(
                    (1 - t) * P.get(position).x + (t) * P.get(position + 1).x,
                    (1 - t) * P.get(position).y + (t) * P.get(position + 1).y
            );
        } else {
            point = new Point(
                    (1 - t) * lerp(t, position, order - 1).x + (t) * lerp(t, position + 1, order - 1).x,
                    (1 - t) * lerp(t, position, order - 1).y + (t) * lerp(t, position + 1, order - 1).y
            );
        }

        return point;
    }

    public double distance() {
        double distance = 0;

        for (int i = 0; i < P.size() - 1; i = i + 1) {
            distance = distance + P.get(i).distanceTo(P.get(i + 1));
        }

        return distance;
    }
}
