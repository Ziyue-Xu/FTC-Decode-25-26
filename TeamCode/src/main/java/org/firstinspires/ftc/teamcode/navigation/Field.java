package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.stuff.SendHelp;

public class Field {

    public int[][] field;

    public Field(int[][] grid) {
        field = SendHelp.clone(grid);
    }

    public void close(int x, int y) {
        field[6 - y][x + 6] = 1;
    }

    public boolean isOpen(int x, int y) {
        System.out.println(6 + y + " = 6 + y");
        System.out.println(x + 6 + " = x + 6");
        return field[6 - y][x + 6] % 2 == 0;
    }

    public int get(int x, int y) {
        return field[6 - y][x + 6];
    }
}
