package frc.team670.robot.constants;

/**
 * All values in meters unless otherwise specified
 * @author Mihir, Marius, Wilson, Radhika, Katia, Kiara
 */
public class FieldConstants {

    // Field Dimensions
    public static final double FIELD_HEIGHT = 8.23;
    public static final double FIELD_WIDTH = 16.46;
    public static final double ORIGIN_TO_CENTER_OF_GOAL = 9.20;

    public static final double FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS = 42.0; // change this later
    public static final double HUB_X_POSITION_INCHES = 324;
    public static final double HUB_X_POSITION_METERS = HUB_X_POSITION_INCHES / 39.37;
    public static final double HUB_Y_POSITION_INCHES = 162;
    public static final double HUB_Y_POSITION_METERS = HUB_Y_POSITION_INCHES / 39.37;
    
    // Hub
    public static final double LOW_HUB_HEIGHT = 1.04;
    public static final double HIGH_HUB_HEIGHT = 2.64;
    public static final double HIGH_PLATE_HEIGHT = 1.71;

    public static final double SMALL_WHATEVER_LEFT_OF_LOGO_ON_PG_26 = 0.34;
    public static final double WIDTH_LOGO = 1.17;

    public static final double LOW_HUB_DIAMETER = 1.53;
    public static final double HIGH_HUB_DIAMETER = 1.22;    
    public static final double HUB_RADIUS = HIGH_HUB_DIAMETER / 2;
    public static final double LOW_TO_FENDER = 0.03;
    public static final double HUB_BASE_WIDTH = 2.72;
    public static final double FENDER_TO_LOW_HUB_OPENING = 0.1;
    public static final double FENDER_TO_CENTER_OF_LOW_HUB = 0.86;
    public static final double HUB_POSE_Y = FIELD_HEIGHT / 2;
    public static final double HUB_POSE_X = FIELD_WIDTH / 2;
    
    //Tarmac
    public static final double TARMAC_TO_EDGE_OF_HUB = 2.15;
    public static final double TARMAC_TO_CENTER_OF_GOAL = 3.51;
    
    //Hangar
    public static final double HANGAR_DISTANCE_Y_FROM_ORIGIN = 5.28;
    public static final double HANGAR_DISTANCE_X_FROM_ORIGIN = 3.27;
    public static final double HANGAR_OPPONENTS_DISTANCE_Y_FROM_ORIGIN = 2.95;
    public static final double HANGAR_OPPONENTS_DISTANCE_X_FROM_ORIGIN = 13.19;
}