package org.firstinspires.ftc.teamcode.stuff;

import org.firstinspires.ftc.teamcode.mathemagics.Point;

public class Ingredient {

    public String type;

    // theta + arm + linear slide
    public double value;
    public Ingredient(String type, double value) {
        this.type = type;
        this.value = value;
    }

    // position
    public Point position;
    public Ingredient(String type, Point position) {
        this.type = type;
        this.position = position;
    }

    // claw
    public String servo;
    public double distance;
    public Ingredient(String type, String servo, double dist) {
        this.type = type;
        this.servo = servo;
        this.distance = dist;
    }
}
