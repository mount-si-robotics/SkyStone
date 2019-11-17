package org.firstinspires.ftc.robotlib.navigation;

public class Point {
    public float x;
    public float y;

    /**
     * Creates a point on the FTC field
     */
    public Point(float x, float y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a point on the FTC field (loss of data when casting to float)
     */
    public Point(double x, double y) {
        this.x = (float) x;
        this.y = (float) y;
    }

    /**
     * Multiplies two points together
     * @param point3D other point3D
     * @return Product
     */
    public double multiply(Point3D point3D) {
        return this.x * point3D.x + this.y * point3D.y;
    }

    /**
     * Calculates the distance between two 2-Dimensional points
     * @param point other point
     * @return Distance
     */
    public double distance(Point point) {
        return Math.sqrt(Math.pow(this.x - point.x, 2) + Math.pow(this.y - point.y, 2));
    }
}