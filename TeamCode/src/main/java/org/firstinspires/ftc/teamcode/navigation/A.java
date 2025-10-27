package org.firstinspires.ftc.teamcode.navigation;

import static org.firstinspires.ftc.teamcode.tuning.Constants.TILE_LENGTH;

import org.firstinspires.ftc.teamcode.mathemagics.Point;

import java.util.PriorityQueue;
import java.util.Stack;

public class A {

    public Field field;

    public A(Field field) {
        this.field = field;
    }

    // "start_point" and "end_point" are Bezier code stuff
    public Stack<Point> path(Point origin, Point destination, Point start_point, Point end_point) {
        ANode begin = new ANode(true, origin.x, origin.y, null);
        ANode end = new ANode(true, destination.x, destination.y, null);

        PriorityQueue<ANode> open_nodes = new PriorityQueue<>();

        open_nodes.add(begin);

        while (!open_nodes.isEmpty()) {
            ANode current_node = open_nodes.poll();

            field.close((int) current_node.x, (int) current_node.y);

            if (current_node.equals(end)) {
                // nodes of the "ideal" path
                Stack<Point> nodes = trace(start_point, current_node, end_point);

                return nodes;
            }

            for (int dx = -1; dx <= 1; dx = dx + 1) {
                for (int dy = -1; dy <= 1; dy = dy + 1) {
                    int x = (int) current_node.x + dx;
                    int y = (int) current_node.y + dy;

                    if (Math.max(Math.abs(x), Math.abs(y)) > 6) {
                        break;
                    }

                    if (field.get(x, y) == 2) {
                        if (dx != 0 && dy != 0) {
                            continue;
                        }
                    }

                    // the next node
                    ANode node = new ANode(true, current_node.x + dx, current_node.y + dy, current_node);

                    // if node is open OR have "become shorter"
                    if (field.isOpen(x, y) && !open_nodes.contains(node)) {
                        node.g = node.distanceTo(begin);
                        node.h = node.distanceTo(end);
                        node.f = node.g + node.h;

                        open_nodes.add(node);
                    } else if (node.distanceTo(begin) <= current_node.g && open_nodes.contains(node)) {
                        open_nodes.remove(node);

                        node.g = node.distanceTo(begin);
                        node.h = node.distanceTo(end);
                        node.f = node.g + node.h;

                        open_nodes.add(node);
                    }
                }
            }
        }

        // you failed, silly
        System.out.println("uh oh");
        return null;
    }

    public Stack<Point> trace(Point start_point, ANode node, Point end_point) {
        Stack<Point> path = new Stack<>();

        path.push(end_point);

        while (node != null) {
            if ((path.peek().x != node.x / 2 * TILE_LENGTH) || (path.peek().y != node.y / 2 * TILE_LENGTH)) {
                if ((node.x / 2 * TILE_LENGTH != start_point.x) || node.x / 2 * TILE_LENGTH != start_point.y) {
                    path.push(new Point(node.x / 2 * TILE_LENGTH, node.y / 2 * TILE_LENGTH));
                }
            }

            node = node.parent;
        }

        path.push(start_point);

        return path;
    }
}
