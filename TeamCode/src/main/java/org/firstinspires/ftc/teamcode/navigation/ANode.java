package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.mathemagics.Point;

public class ANode extends Point implements Comparable<ANode> {

    public boolean open;

    public double g;
    public double h;
    public double f;

    public ANode parent;

    public ANode(boolean open, double x, double y, ANode parent) {
        super(x, y);

        this.open = open;
        this.parent = parent;
    }

    @Override
    public int compareTo(ANode a) {
        if (f < a.f) {
            return -1;
        } else if (f == a.f) {
            if (h < a.h) {
                return -1;
            } else if (h == a.h) {
                return 0;
            } else if (h > a.h) {
                return 1;
            }
        } else if (f > a.f) {
            return 1;
        }

        // nuuuuull!
        return 69420;
    }
}
