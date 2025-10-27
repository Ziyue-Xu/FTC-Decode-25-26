package org.firstinspires.ftc.teamcode.stuff;

// a remedy for java's skill issues

public class SendHelp {

    /**
     * Clone a 2d array
     * @param array
     * @return
     */
    public static int[][] clone(int[][] array) {
        int[][] result = new int[array.length][array[0].length];

        for (int i = 0; i < array.length; i = i + 1) {
            for (int j = 0; j < array[i].length; j = j + 1) {
                result[i][j] = array[i][j];
            }
        }

        return result;
    }
}
