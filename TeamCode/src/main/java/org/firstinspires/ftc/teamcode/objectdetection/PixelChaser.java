package org.firstinspires.ftc.teamcode.objectdetection;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.pathfinder.PFinder;
import org.firstinspires.ftc.teamcode.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.PixelDetectionPipeline;

public class PixelChaser {

    private PFinder pFinder;
    private boolean isToggled = false;
    private PixelDetectionPipeline.Pixel latestRegion;
    private PIDController rotController = new PIDController(0.1, 0, 0);
    private PIDController driveController = new PIDController(0.1, 0, 0);
    private double rotPower = 0;
    private double drivePower = 0;

    //Placeholder
    private static double areaAtTargetDistance = 30;

        public PixelChaser(PixelDetectionPipeline.Pixel latestRegion, PFinder pFinder ) {

            this.pFinder = pFinder;
            this.latestRegion = latestRegion;


        }

        public void loop() {


            if (isToggled && latestRegion != null) {
                rotPower = rotController.calculate(latestRegion.center.x);
                drivePower = driveController.calculate(areaAtTargetDistance-latestRegion.boundingRect.area());

                pFinder.drive(new Pose( drivePower * 0.5, 0, rotPower*0.5));

                if(Math.abs(rotPower) < 0.1 && Math.abs(drivePower) < 0.1){
                    isToggled = false;
                }

            }

        }

        public void toggle() {
            isToggled = !isToggled;
        }
}


