package org.firstinspires.ftc.teamcode.apriltag;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;
import org.openftc.apriltag.AprilTagDetection;

public class TagChaser {
    private AprilTagCamera camera;
    private PFinder pFinder;
    private boolean isToggled = false;
    private AprilTagDetection latestDetection;
    private PIDController rotController = new PIDController(0.1, 0, 0);
    private PIDController yController = new PIDController(0.1, 0, 0);
    private PIDController xController = new PIDController(0.1, 0, 0);

    private double rotPower = 0;
    private double xPower = 0;
    private double yPower = 0;
    private double specifiedID = 0;



    public TagChaser(AprilTagCamera camera, PFinder pFinder) {
        this.camera = camera;
        this.pFinder = pFinder;
    }

    public void loop() {
        latestDetection = camera.getLastDetection();

        if (isToggled && latestDetection != null && latestDetection.id == specifiedID){
            rotPower = rotController.calculate(latestDetection.pose.z);
            xPower = yController.calculate(latestDetection.pose.x);
            yPower = xController.calculate(latestDetection.pose.y);

            pFinder.drive(new Pose( xPower * 0.5, yPower * 0.5, rotPower*0.5));

            if(rotPower < 0.1 && xPower < 0.1 && yPower < 0.1){
                isToggled = false;
            }

        }

    }

    public void toggle() {
        isToggled = !isToggled;
    }

    public void setID(double id){
        specifiedID = id;
    }
}
