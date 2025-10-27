package org.firstinspires.ftc.teamcode.mechanics;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.TeleOp.Main;

public class ServoErmSigma {
    //     Ports: 0         1           2         3
    public static Servo trans;
    public static Servo hood;



    //public double intakeLeft = 300, intakeRight = 0, outtakeLeft = 200, outtakeRight = 100;

    public ServoErmSigma(HardwareMap hardwareMap){
        trans = hardwareMap.servo.get("trans");
        hood = hardwareMap.servo.get("hood");
    }

    public void transP1() {
        trans.setPosition(.9);
    }

    public void transP2() {
        trans.setPosition(0.5);
    }

    public void setHood(double dist) {
        hood.setPosition(dist);
    }


}