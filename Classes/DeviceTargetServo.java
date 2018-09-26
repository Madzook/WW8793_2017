package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * DeviceTargetServo Class
 * Created 11/11/2017 for the Greenwood FTC Robotics Club.
 * This class is designed for non-continuous servos
 * Objects created from this class will have the following status and behaviors:
 *    Status:
 *        + targetEngUnits = Desired final target of servo
 *        + currentPosition = Position currently commanded to the servo
 *    Behavior:
 *        + incrementPosition - Adjust servo position by a single increment
 *        + goToPosition - Adjust servo incrementally until the target position is achieved
 *        + goToPositionNow - Immediately set servo position to the target position
 *    Interfaces:
 *        + targetServo - Servo interface
 *
 * Revised 18Nov2017 */



public class DeviceTargetServo
{
    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * ------------------------------------------------------- */
    public Servo
            targetServo;     // Servo to be monitored and controlled

    public double
            targetPosition,  // Desired final target of servo (0-1.0 = 0-180 degrees)
            currentPosition; // Current position commanded to servo (0-1.0 = 0-180 degrees)

    public String
            name;


    /* -------------------------------------------------------
     * Private (Concealed) Class Members
     * -- none --
     * ------------------------------------------------------- */



    /* =======================================================
     * CLASS CONSTRUCTOR
     * -------------------------------------------------------
     * Purpose: Establish and initialize a new TargetServo (i.e., non-continuous servo)
     * Input Parameters:
     *    + hwMap = Robot hardware
     *    + servoName = Servo name as configured on the Android device via the Robot Controller app
     *    + servoInitialPosition = Initial position of the servo hub (0-1.0 = 0-180 degrees)
     * Operations:
     *   + Link targetServo to the actual installed servo given the servo name as configured on the Android phone.
     *   + Initialize public variables targetServo and currentPosition.
     *   + Set the initial position of the new servo object (0-1.0 = 0-180 degrees)
     * ======================================================= */
    DeviceTargetServo(HardwareMap hwMap, String servoName, double servoInitialPosition) {
        name = servoName;                         // Save name for later reference
        targetServo = hwMap.servo.get(name);      // Link servo to installed hardware
        targetPosition = servoInitialPosition;    // Set initial value for desired target
        currentPosition = servoInitialPosition;   // Set initial value for current set position
        targetServo.setPosition(currentPosition); // Command the servo to go to the new 'current position' (i.e., final target position)
    }


    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: incrementPosition
     * Purpose: Increment the servo closer to the destination target
     * Note: The method needs to be placed in a loop for the desired target to be realized.
     * Input Parameters:
     *    + servoTarget = Desired final position (0-1.0 = 0-180 degrees)
     *    + servoIncPos = Servo position step change allowed per program scan (0-1.0000 = 0-180.00)
     * Return:
     *    + true = Desired servo position achieved
     *    + false = Servo not at target
     * ------------------------------------------------------- */
    public boolean incrementPosition(double servoTarget, double servoIncPos) {

        targetPosition = servoTarget;  // Desired target

        if (targetPosition > currentPosition) {   // If desired target is greater than the current position, INCREMENT current position
            currentPosition = currentPosition + Math.abs(servoIncPos);  // Math.abs is the 'absolute value calculation' method in the "Math" class

            if (currentPosition > targetPosition) // If the pending incremented current position exceeds target position, set current position equal to target position
                currentPosition = targetPosition;
        }

        if (targetPosition < currentPosition) {   // If desired target is less than the current position, DECREMENT current position
            currentPosition = currentPosition - Math.abs(servoIncPos);

            if (currentPosition < targetPosition) // If the pending decremented current position is less than the target position, set current position equal to target position
                currentPosition = targetPosition;
        }

        targetServo.setPosition(currentPosition); // Command the servo to go to the new 'current position'

        return (currentPosition == targetPosition); // Return true if target position acheived.
    }


    /* -------------------------------------------------------
     * Method: goToPosition
     * Purpose: Send the servo to the target position at a controlled speed (increment per program scan)
     * Note: The method employs the "incrementPosition" method in "this" class to achieve target position
     * Input Parameters:
     *    + servoTarget = Desired final position (0-1.0 = 0-180 degrees)
     *    + servoIncPos = Servo position step change allowed per program scan (0-1.0000 = 0-180.00)
     * ------------------------------------------------------- */
    public void goToPosition (double servoTarget, double servoIncPos) {
        while (!this.incrementPosition(servoTarget, servoIncPos)); // Loop until the desired target position is achieved.
    }


    /* -------------------------------------------------------
     * Method: goToPositionNow
     * Purpose: Command the servo to go immediately to the target position.
     * Input Parameters:
     *    + servoTarget = Desired final position (0-1.0 = 0-180 degrees)
     * ------------------------------------------------------- */
    public void goToPositionNow(double servoTarget) {
        targetPosition = servoTarget;              // Update desired target and current position variables (public status)
        currentPosition = targetPosition;
        targetServo.setPosition(currentPosition); // Command the servo to go to the new 'current position' (i.e., target position)
    }

}  // END DeviceTargetServo
