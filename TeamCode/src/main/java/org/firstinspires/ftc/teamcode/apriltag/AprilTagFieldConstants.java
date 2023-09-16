package org.firstinspires.ftc.teamcode.apriltag;

import me.wobblyyyy.pathfinder2.geometry.PointXYZ;

public class AprilTagFieldConstants {

    //ALL Placeholder!!!
    public static PointXYZ TAG_0_POSE = new PointXYZ(0,0,0);
    public static PointXYZ TAG_1_POSE = new PointXYZ(1,1,0);
    public static PointXYZ TAG_2_POSE = new PointXYZ(1,2,0);
    public static PointXYZ TAG_3_POSE = new PointXYZ(2,2,0);
    public static PointXYZ TAG_4_POSE = new PointXYZ(3,2,90);
    public static PointXYZ TAG_5_POSE = new PointXYZ(1,2,90);
    public static PointXYZ TAG_6_POSE = new PointXYZ(1,2,90);
    public static PointXYZ TAG_7_POSE = new PointXYZ(1,0,90);
    public static PointXYZ TAG_8_POSE = new PointXYZ(5,2,90);
    public static PointXYZ TAG_9_POSE = new PointXYZ(5,3,90);
    public static PointXYZ TAG_10_POSE = new PointXYZ(5,3,180);
    public static PointXYZ TAG_11_POSE = new PointXYZ(5,3,180);
    public static PointXYZ TAG_12_POSE = new PointXYZ(3,3,180);
    public static PointXYZ TAG_13_POSE = new PointXYZ(3,2,180);
    public static PointXYZ TAG_14_POSE = new PointXYZ(3,1,180);
    public static PointXYZ TAG_15_POSE = new PointXYZ(3,0,180);
    public static PointXYZ TAG_16_POSE = new PointXYZ(3,0,270);
    public static PointXYZ TAG_17_POSE = new PointXYZ(3,1,270);
    public static PointXYZ TAG_18_POSE = new PointXYZ(3,2,270);
    public static PointXYZ TAG_19_POSE = new PointXYZ(3,3,270);

    public static PointXYZ getTagPose(int id) {
        switch (id) {
            case 0:
                return TAG_0_POSE;
            case 1:
                return TAG_1_POSE;
            case 2:
                return TAG_2_POSE;
            case 3:
                return TAG_3_POSE;
            case 4:
                return TAG_4_POSE;
            case 5:
                return TAG_5_POSE;
            case 6:
                return TAG_6_POSE;
            case 7:
                return TAG_7_POSE;
            case 8:
                return TAG_8_POSE;
            case 9:
                return TAG_9_POSE;
            case 10:
                return TAG_10_POSE;
            case 11:
                return TAG_11_POSE;
            case 12:
                return TAG_12_POSE;
            case 13:
                return TAG_13_POSE;
            case 14:
                return TAG_14_POSE;
            case 15:
                return TAG_15_POSE;
            case 16:
                return TAG_16_POSE;
            case 17:
                return TAG_17_POSE;
            case 18:
                return TAG_18_POSE;
            case 19:
                return TAG_19_POSE;
            default:
                return null;
        }
    }



}
