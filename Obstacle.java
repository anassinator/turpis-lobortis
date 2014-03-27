/**
 * Contains obstacle information
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.5
 */
public class Obstacle {
    // OBSTACLE TYPES
    public final int IDK = -1;
    public final int WOOD = 0;
    public final int LIGHT_BLUE = 1;
    public final int RED = 2;
    public final int YELLOW = 3;
    public final int WHITE = 4;
    public final int DARK_BLUE = 5;

    public int id;
    public double[] position = new double[2];

    private Odometer odometer;

    /**
     * Obstacle constructor
     *
     * @param id            id specifying whether it's an obstacle or a flag
     * @param distance      distance from obstacle
     * @param odometer      odometer object containing robot's position and orientation
     */
    public Obstacle(int id, double distance, Odometer odometer) {
        this.id = id;
        this.odometer = odometer;

        this.position[0] = odometer.getX() + distance * Math.cos(odometer.getTheta());
        this.position[1] = odometer.getY() + distance * Math.sin(odometer.getTheta());
    }

}
