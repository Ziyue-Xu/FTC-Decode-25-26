package org.firstinspires.ftc.teamcode.mechanics;

// all of teamcode

import org.firstinspires.ftc.teamcode.stuff.Robot;

/* NOTES:
	mecanum controls

	left_stick_x -> turning
		~ negative = counter clockwise
		~ positve = clockwise

	right_stick_y = forward/backward
		~ positive = forward
		~ negative = backward

	right_stick_x = strafing ooo
		~ negative = leftward
		~ positive = rightward
*/

public class Drivetrain {

    public static double PERCENT_POWER = 1;

    public static double y_power = 0;
    public static double x_power = 0;
    public static double final_x_power = 0;
    public static double final_y_power = 0;
    public static double rotational_power = 0;

    public static double front_left_motor_power = 0;
    public static double front_right_motor_power = 0;
    public static double back_left_motor_power = 0;
    public static double back_right_motor_power = 0;

    // field centric mode?
    public static boolean FIELD_CENTRIC;

    public static void robot_centric_drive(double y_power, double x_power, double rotational_power) {
        // Note: The "correction" is to counter potential errors in strafing
        // Tip: Let the driver team tune this value
        double correction = 1;
        x_power = x_power * correction;

        // Rotating the robot
        // negative = counter clockwise
        // positive = clockwise

        // x, y, rotational -> motor powers
        front_left_motor_power = y_power + x_power + rotational_power;
        back_right_motor_power = y_power + x_power - rotational_power;

        front_right_motor_power = y_power - x_power - rotational_power;
        back_left_motor_power = y_power - x_power + rotational_power;

        // tricky part: since the motors take input on the interval [-1, 1]
        // we want to "normalize" the powers to scale them by
        // dividing by the sum of the magnitude of the powers

        y_power = Math.abs(y_power);
        x_power = Math.abs(x_power);
        rotational_power = Math.abs(rotational_power);

        // dividing by zero = big no no
        double magnitude_sum = Math.max(1D, y_power + x_power + rotational_power);

        front_left_motor_power = front_left_motor_power / magnitude_sum;
        back_right_motor_power = back_right_motor_power / magnitude_sum;

        front_right_motor_power = front_right_motor_power / magnitude_sum;
        back_left_motor_power = back_left_motor_power / magnitude_sum;

        // Supply derived power to motors
        Robot.front_left_motor.setPower(front_left_motor_power);
        Robot.back_right_motor.setPower(back_right_motor_power);

        Robot.front_right_motor.setPower(front_right_motor_power);
        Robot.back_left_motor.setPower(back_left_motor_power);
    }

    public static void field_centric_drive(double y_power, double x_power, double rotational_power) {
        // "field centric y power"
        final_y_power = x_power * -Math.sin(Robot.theta - Robot.initial_theta) + y_power * Math.cos(Robot.theta - Robot.initial_theta);

        // "field centric x power"
        final_x_power = x_power * Math.cos(Robot.theta - Robot.initial_theta) - y_power * -Math.sin(Robot.theta - Robot.initial_theta);

        // Note: The "correction" is to counter potential errors in strafing
        // Tip: Let the driver team tune this value
        double correction = 1;
        final_x_power = final_x_power * correction;

        // Rotating the robot
        // negative = counter clockwise
        // positive = clockwise

        // x, y, rotational -> motor powers
        double front_left_motor_power = final_y_power + final_x_power + rotational_power;
        double back_right_motor_power = final_y_power + final_x_power - rotational_power;

        double front_right_motor_power = final_y_power - final_x_power - rotational_power;
        double back_left_motor_power = final_y_power - final_x_power + rotational_power;

        // tricky part: since the motors take input on the interval [-1, 1]
        // we want to "normalize" the powers to scale them by
        // dividing by the sum of the magnitude of the powers

        final_y_power = Math.abs(final_y_power);
        final_x_power = Math.abs(final_x_power);
        rotational_power = Math.abs(rotational_power);

        // dividing by zero = big no no
        double magnitude_sum = Math.max(1D, final_y_power + final_x_power + rotational_power);

        front_left_motor_power = front_left_motor_power / magnitude_sum;
        back_right_motor_power = back_right_motor_power / magnitude_sum;

        front_right_motor_power = front_right_motor_power / magnitude_sum;
        back_left_motor_power = back_left_motor_power / magnitude_sum;

        // Supply derived power to motors
        Robot.front_left_motor.setPower(front_left_motor_power);
        Robot.back_right_motor.setPower(back_right_motor_power);

        Robot.front_right_motor.setPower(front_right_motor_power);
        Robot.back_left_motor.setPower(back_left_motor_power);
    }

    public static void absolute_drive(double y_power, double x_power, double rotational_power) {
        // "absolute y power"
        final_y_power = x_power * -Math.sin(Robot.theta - Math.PI / 2) + y_power * Math.cos(Robot.theta - Math.PI / 2);

        // "field centric x power"
        final_x_power = x_power * Math.cos(Robot.theta - Math.PI / 2) - y_power * -Math.sin(Robot.theta - Math.PI / 2);

        // Note: The "correction" is to counter potential errors in strafing
        // Tip: Let the driver team tune this value
        double correction = 1;
        final_x_power = final_x_power * correction;

        // Rotating the robot
        // negative = counter clockwise
        // positive = clockwise

        // x, y, rotational -> motor powers
        front_left_motor_power = final_y_power + final_x_power + rotational_power;
        back_right_motor_power = final_y_power + final_x_power - rotational_power;

        front_right_motor_power = final_y_power - final_x_power - rotational_power;
        back_left_motor_power = final_y_power - final_x_power + rotational_power;


        // tricky part: since the motors take input on the interval [-1, 1]
        // we want to "normalize" the powers to scale them by
        // dividing by the sum of the magnitude of the powers

        final_y_power = Math.abs(final_y_power);
        final_x_power = Math.abs(final_x_power);
        rotational_power = Math.abs(rotational_power);

        // dividing by zero = big no no
        double magnitude_sum = Math.max(1D, final_y_power + final_x_power + rotational_power);

        front_left_motor_power = front_left_motor_power / magnitude_sum;
        back_right_motor_power = back_right_motor_power / magnitude_sum;

        front_right_motor_power = front_right_motor_power / magnitude_sum;
        back_left_motor_power = back_left_motor_power / magnitude_sum;

        System.out.println("frontL power = " + front_left_motor_power + " frontR power = " + front_right_motor_power + " backL power = " + back_left_motor_power + " backR power = " + back_right_motor_power);

        // Supply derived power to motors
        Robot.front_left_motor.setPower(front_left_motor_power);
        Robot.back_right_motor.setPower(back_right_motor_power);

        Robot.front_right_motor.setPower(front_right_motor_power);
        Robot.back_left_motor.setPower(back_left_motor_power);
    }

    // simulate acceleration
    public static void accelerate(double stick_y, double stick_x) {
        // Note: Y stick values are reversed
        // Old -> Up is negative; Down is positive
        // New -> Up is positive; Down is negative

        Drivetrain.y_power = Drivetrain.y_power - stick_y;
        Drivetrain.x_power = Drivetrain.x_power + stick_x;
    }
}