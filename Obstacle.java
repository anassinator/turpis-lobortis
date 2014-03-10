/**
 * Obstacle.java
 * Contains obstacle information
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */

public class Obstacle {
    private final int OBSTACLE = 0;
    private final int FLAG = 1;

    public int id;
    public double[] position = new double[2];

    private Odometer odometer;

    /**
     * Obstacle constructor
     * @param id specifying whether it's an obstacle or a flag
     * @param distance from obstacle
     * @param odometer object containing robot's position and orientation
     */
    public Obstacle(int id, double distance, Odometer odometer) {
        this.id = id;
        this.odometer = odometer;

        this.position[0] = odometer.getX() + distance * Math.cos(odometer.getTheta());
        this.position[1] = odometer.getY() + distance * Math.sin(odometer.getTheta());
    }

}
