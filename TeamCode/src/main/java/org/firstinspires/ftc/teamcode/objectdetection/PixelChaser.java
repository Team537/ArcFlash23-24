package org.firstinspires.ftc.teamcode.objectdetection;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;

public class PixelChaser {
    private DetectorCamera camera;
    private PFinder pFinder;
    private boolean isToggled = false;
    private ColorDetectionPipeline.ColorRegion latestRegion;
    private PIDController rotController = new PIDController(0.1, 0, 0);
    private PIDController driveController = new PIDController(0.1, 0, 0);
    private double rotPower = 0;
    private double drivePower = 0;

    //Placeholder
    private static double areaAtTargetDistance = 30;

        public PixelChaser(DetectorCamera camera, PFinder pFinder) {
            this.camera = camera;
            this.pFinder = pFinder;
        }

        public void loop() {
            latestRegion = camera.getLastDetectedRegion();

            if (isToggled && latestRegion != null) {
                rotPower = rotController.calculate(latestRegion.relativePosition.x);
                drivePower = driveController.calculate(areaAtTargetDistance-latestRegion.boundingRect.area());

                pFinder.teleopDrive(new Pose( drivePower * 0.5, 0, rotPower*0.5));

                if(rotPower < 0.1 && drivePower < 0.1){
                    isToggled = false;
                }

            }

        }

        public void toggle() {
            isToggled = !isToggled;
        }
}


