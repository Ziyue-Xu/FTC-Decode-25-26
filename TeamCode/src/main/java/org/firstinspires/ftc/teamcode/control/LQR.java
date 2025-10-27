package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.mathemagics.Matrix;
import org.firstinspires.ftc.teamcode.mathemagics.Oikura;

import java.util.ArrayList;

public class LQR {

    // discrete-time linear (state-space) system
    /*
        x_t = A_{t - 1} x_{t - 1} + B_{t - 1} u_{t - 1}
    */

    // state matrix
    public double[][] A;
    // control/input matrix
    public double[][] B;

    // state-cost weighted matrix
    public double[][] Q;
    // control/input-cost weighted matrix
    public double[][] R;

    public LQR(double[][] A, double[][] B, double[][] Q, double[][] R) {
        this.A = A;
        this.B = B;
        this.Q = Q;
        this.R = R;
    }

    // process variable, x, (PV)
    // setpoint, x_f, (SP)

    public double[][] control(double[][] x, double[][] x_f) {
        // error = PV - SP
        double[][] e = Matrix.subtract(x, x_f);

        int N = 52;

        // P = solution to the discrete-time algebraic Riccati equation (DARE)

        ArrayList<double[][]> P = new ArrayList<>();
        while(P.size() < N + 1) P.add(new double[][]{{}});

        P.set(N, Q);

        for (int i = N; i >= 1; i = i - 1) {
            P.set(i - 1, Oikura.DARE(A, B, Q, R, P.get(i)));
        }

        // K = optimal feedback gains (vector)

        ArrayList<double[][]> K = new ArrayList<>();
        while(K.size() < N) K.add(new double[][]{{}});

        for (int i = 0; i < N; i = i + 1) {
            K.set(i, Oikura.K(A, B, R, P.get(i + 1))) ;
        }

        // u = optimal control/input

        ArrayList<double[][]> u = new ArrayList<>();
        while(u.size() < N) u.add(new double[][]{{}});

        for (int i = 0; i < N; i = i + 1) {
            u.set(i, Oikura.u(K.get(i), e));
        }

        return u.get(N - 1);
    }
}
