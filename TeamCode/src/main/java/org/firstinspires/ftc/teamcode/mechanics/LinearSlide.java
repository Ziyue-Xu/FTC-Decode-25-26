//package org.firstinspires.ftc.teamcode.mechanics;//package org.firstinspires.ftc.teamcode.mechanics;
//
//import org.firstinspires.ftc.teamcode.control.*;
//import static org.firstinspires.ftc.teamcode.stuff.Robot.*;
//
//import static org.firstinspires.ftc.teamcode.tuning.Constants.*;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//public class LinearSlide {
//
//    public static PID pid = null;
//
//    public static double POSITION;
//
//    /**
//     *
//     * @param position in meters
//     */
//    public static boolean go_to(double position) {
//        double target = position * CPR / (.02 * Math.PI * 3);
//        slide1.setTargetPosition((int) (target));
//        slide1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slide1.setPower(1);
//        slide2.setTargetPosition((int) (target));
//        slide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slide2.setPower(1);
//
//        System.out.println(slide1.getCurrentPosition() + " slide 1");
//        System.out.println(slide2.getCurrentPosition() + " slide 2");
//
//        System.out.println(position);
//        POSITION = (slide1.getCurrentPosition() + slide2.getCurrentPosition())/2.0;
////        if (Math.abs((slide1.getCurrentPosition() + slide2.getCurrentPosition())/2.0 - target) <= 10) {
////            slide1.setPower(0);
////            slide2.setPower(0);
////        }
//
//        return (Math.abs(LinearSlide.POSITION - position) < 0.01);
//    }
//}
