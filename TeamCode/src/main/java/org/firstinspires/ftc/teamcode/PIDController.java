package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP; // Proportional constant
    private double kI; // Integral constant
    private double kD; // Derivative constant

    private double integral;
    private double previousError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double error) {
        // Proportional term
        double proportional = kP * error;

        // Integral term
        integral += error;
        double integralTerm = kI * integral;

        // Derivative term
        double derivative = (error - previousError);
        double derivativeTerm = kD * derivative;

        // Update previous error
        previousError = error;

        // Calculate PID output
        return proportional + integralTerm + derivativeTerm;
    }
}
