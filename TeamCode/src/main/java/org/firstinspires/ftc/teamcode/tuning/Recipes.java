package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.tuning.Constants.LENGTH;
import static org.firstinspires.ftc.teamcode.tuning.Constants.TILE_LENGTH;

import org.firstinspires.ftc.teamcode.mathemagics.Point;
import org.firstinspires.ftc.teamcode.stuff.Ingredient;

public class Recipes {

    // LET US COOK! 🗣‼️

    public static Point desired_position;
    public static double desired_theta;
    public static double desired_linear_slide_position = 0;
    public static double desired_linkage_length;
    public static double desired_arm_angle;

    // autonomous recipes

    public static final Ingredient[][] redGoalSide = {
            {
                    new Ingredient("position", new Point(-1.8 * TILE_LENGTH, -.2 * TILE_LENGTH)),
                    // same servo values as y in teleop
                    new Ingredient("servo", "hood", 2),
                    new Ingredient("linear slide", .25)
            },

    };

    public static final Ingredient[][] lqr = {


            {
                    new Ingredient("position", new Point(-1.8 * TILE_LENGTH, -.2 * TILE_LENGTH))
            },

    };


//    public static final Ingredient[][] cycle = {
//        {
//            new Ingredient("arm", Math.toRadians(170))
//        },
//
//        {
//            // varies
//            new Ingredient("position", new Point(0, 0)),
//            new Ingredient("theta", Math.toRadians(90.0)),
//            new Ingredient("linear slide", 0.3),
//        },
//
//        {
//            new Ingredient("claw", "close right", "skull"),
//        },
//
//        {
//            new Ingredient("linear slide", 0.075),
//            // varies
//            new Ingredient("position", new Point(0, 0))
//        },
//
//        {
//            new Ingredient("arm", Math.toRadians(30))
//        },
//
//        {
//            new Ingredient("linear slide", 0.5)
//        },
//    };
}
