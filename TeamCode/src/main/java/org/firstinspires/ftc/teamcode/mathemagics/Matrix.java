package org.firstinspires.ftc.teamcode.mathemagics;

// ;-;
import org.apache.commons.math3.linear.MatrixUtils;

public class Matrix {

    public static double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];

        for (int m = 0; m < A.length; m = m + 1) {
            for (int n = 0; n < A[m].length; n = n + 1) {
                C[m][n] = A[m][n] + B[m][n];
            }
        }

        return C;
    }

    public static double[][] subtract(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];

        for (int m = 0; m < A.length; m = m + 1) {
            for (int n = 0; n < A[m].length; n = n + 1) {
                C[m][n] = A[m][n] - B[m][n];
            }
        }

        return C;
    }

    public static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = new double[A.length][B[0].length];

        for (int a_row = 0; a_row < A.length; a_row = a_row + 1) {
            for (int b_column = 0; b_column < B[0].length; b_column = b_column + 1) {
                for (int a_column = 0; a_column < A[0].length; a_column = a_column + 1) {
                    C[a_row][b_column] = C[a_row][b_column] + A[a_row][a_column] * B[a_column][b_column];
                }
            }
        }

        return C;
    }

    public static double[][] multiply(double A, double[][] B) {
        double[][] C = new double[B.length][B[0].length];

        for (int m = 0; m < C.length; m = m + 1) {
            for (int n = 0; n < C[m].length; n = n + 1) {
                C[m][n] = A * B[m][n];
            }
        }

        return C;
    }

    public static double[][] transpose(double[][] A) {
        double[][] A_T = new double[A[0].length][A.length];

        for (int m = 0; m < A.length; m = m + 1) {
            for (int n = 0; n < A[m].length; n = n + 1) {
                A_T[n][m] = A[m][n];
            }
        }

        return A_T;
    }

    public static double[][] inverse(double[][] A) {
        // honestly, i can't be bothered to write a O(n^3) algorithm for this :skull:

        return MatrixUtils.inverse(MatrixUtils.createRealMatrix(A)).getData();
    }
}
