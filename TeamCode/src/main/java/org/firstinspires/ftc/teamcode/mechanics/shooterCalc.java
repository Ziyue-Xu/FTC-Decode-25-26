package org.firstinspires.ftc.teamcode.mechanics;

public class shooterCalc {
    public static double distToAngle(double dist) {
        if (dist >= 1.9 && dist < 2.2) {
            return 1;
        }
        else if (dist >= 2.2 && dist < 2.3){
            return .7;
        }
        else if (dist >= 1.27 && dist < 1.76) {
            return .5;
        }
        else if (dist >= 1.76 && dist < 1.97) {
            return .41;
        }
        else if (dist >= 1.97 && dist < 2.515) {
            return .3;
        }
        else if (dist >= 2.515 && dist < 2.71) {
            return .25;
        }
        else if (dist >= 2.71 && dist < 2.975) {
            return .15;
        }
        else {
            return -.00151515151515 * dist + .80152;
        }
    }
}
