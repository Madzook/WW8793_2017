package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;


/**
 * DeviceColorSensor Class
 * Created 11Nov2017 for the Greenwood FTC Robotics Club.
 * This class is designed for Modern Robotics or equivalent color sensors with integral LED.
 *
 * Objects created from this class will have the following status and behaviors:
 *    Status:
 *        + hue, saturation, value = floating point color attributes
 *        + colorRed, colorGreen, colorBlue, colorWhite = boolean color evaluation
 *    Behavior:
 *        + turnOnLight - Turn on integral color sensor LED
 *        + turnOffLight - Turn off LED
 *        + evaluateColor - Solve for hue, saturation, and value, and set colorRed, colrGreen, colorBlue and colorWhite accordingly
 *        + isWhite - Determine if white color is present and update colorWhite accordingly
 *    Interfaces:
 *        + colorDetector - ColorSensor interface
 *
 * Revised 15Dec2017 */


public class DeviceColorSensor {

    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * ------------------------------------------------------- */
    public ColorSensor
            colorDetector;    // Color sensor to be monitored and integral LED to be controlled

    public String
            name;

    public float
            // Color results (hue, saturation, and value.
            hue,          // Hue (0-360 degrees)
            saturation,
            value;

    public int
            i2cAddress;  // Color sensor I2C bus address

    public boolean
            colorRed,   // Color Red detected status (true=present)
            colorGreen, // Color Green detected status (true=present)
            colorBlue,  // Color Blue detected status (true=present)
            colorWhite, // Color white detected status (true=present)
            lightOn;    // LED On/Off status (true=ON)


    /* -------------------------------------------------------
     * Private (Concealed) Class Members
     * ------------------------------------------------------- */
    private static final float
            /* Minimum and maximum hue, saturation, and value ranges for colors red, green, and blue.
             * Note: Limits may need to be updated specific to the competition's field elements. */
            MIN_RED_HUE = 330.0f,   // On the hue 'circle', red is 0 degrees.
            MAX_RED_HUE = 30.0f,
            MIN_RED_SAT = 0.6f,
            MIN_RED_VALUE = 0.025f,
            MIN_GREEN_HUE = 90.0f,  // On the hue circle, green is 120 degrees
            MAX_GREEN_HUE = 150.0f,
            MIN_GREEN_SAT = 0.5f,
            MIN_GREEN_VALUE = 0.3f,
            MIN_BLUE_HUE = 210.0f,  // On the hue circle, blue is 240 degrees
            MAX_BLUE_HUE = 270.0f,
            MIN_BLUE_SAT = 0.6f,
            MIN_BLUE_VALUE = 0.025f;

    private static final int
            // Minimum alpha value for color white
            MIN_WHITE_ALPHA = 20; // Minimum brightness for a white line

    private float[]
            hsvResult = {0f, 0f, 0f}; /* Array of intermediate color sensor results (HSV)
                                         An array is required for use by the Color.RGBToHSV() method */


    /* =======================================================
     * CLASS CONSTRUCTOR
     * -------------------------------------------------------
     * Purpose: Establish and initialize a new TargetServo (i.e., non-continuous servo)
     * Input Parameters:
     *    + hwMap = Robot hardware
     *    + colorSensorName = Color sensor name as configured on the Android device via the Robot Controller app
     *    + colorSensorI2cAddress = The unique I2C address for the color sensor
     *    + lightON = The initial status of the coloro sensor light (true=ON)
     * Operations:
     *   + Link colorDetector to the actual installed color sensor given the sensor name as configured on the Android phone,
     *           and the sensor's unique I2C address.
     *   + Initialize lightOn.
     *   + As applicable, turn on the light (i.e., LED).
     * ======================================================= */
    DeviceColorSensor(HardwareMap hwMap, String sensorName, int sensorI2cAddress) {
        name = sensorName;                            // Capture color sensor name (The construct parameter, sensorName, could have been used in the next line,
                                                      //          instead of the declared String, name, but this allows the name to be recalled (e.g., telemetry)
        colorDetector = hwMap.colorSensor.get(name);  // Link colorDetector to installed hardware via name
        i2cAddress = sensorI2cAddress;                // Capture color sensor I2C address for subsequent sensor calibration. Unfortunately, the I2C address cannot
    }                                                 //          be established in the constructor.


    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: initialize
     * Purpose: Complete initialization of the color sensor by identifying the sensors I2C bus address and turning off the light
     * Note: Other teams have noticed that Modern Robitics color sensors may not operate correctly if the LED is turned ON immediately following calibration.
     *       This has not been an issue for Greenwood, but no need to risk fate.
     * ------------------------------------------------------- */
    public void initialize() {
        colorDetector.setI2cAddress(I2cAddr.create8bit(i2cAddress)); // Link colorDetector to installed hardware via address
        // this.turnOffLight();  // Turn off color sensor ID using method turnOffLight in "this" class.
    }


    /* -------------------------------------------------------
     * Method: turnOnLight
     * Purpose: Turn on the color sensor LED and update the corresponding light status variable
     * ------------------------------------------------------- */
    public void turnOnLight () {
        lightOn = true;
        colorDetector.enableLed(lightOn);
    }


    /* -------------------------------------------------------
     * Method: turnOffLight
     * Purpose: Turn off the color sensor LED and update the corresponding light status variable
     * ------------------------------------------------------- */
    public void turnOffLight () {
        lightOn = false;
        colorDetector.enableLed(lightOn);
    }


    /* -------------------------------------------------------
     * Method: evaluateColor
     * Purpose: Calculate hue, saturation, and value, and identify color (Red, Green, Blue, White)
     *              colorRed status (true = red)
     *              colorGreen status (true = green)
     *              colorBlue status (true = blue)
     *              colorWhite status (true = white)
     * Return: true = Red, Green, Blue, White detected
     * ------------------------------------------------------- */
    public boolean evaluateColor () {
        // Convert RGB to HSV via the Color.RGBToHSV() method.
        Color.RGBToHSV(colorDetector.red() * 8, colorDetector.green() * 8, colorDetector.blue() * 8, hsvResult);

        // Harvest HSV from the hsvResult array
        hue = hsvResult[0];
        saturation = hsvResult[1];
        value = hsvResult[2];

        // Given hue, saturation, and value, determine if color Red, Green, and Blue are present
        colorRed = ((hue >= MIN_RED_HUE) || (hue <= MAX_RED_HUE)) && (saturation >= MIN_RED_SAT) && (value >= MIN_RED_VALUE);
        colorGreen = (hue >= MIN_GREEN_HUE) && (hue <= MAX_GREEN_HUE) && (saturation >= MIN_GREEN_SAT) && (value >= MIN_GREEN_VALUE);
        colorBlue = (hue >= MIN_BLUE_HUE) && (hue <= MAX_BLUE_HUE) && (saturation >= MIN_BLUE_SAT) && (value >= MIN_BLUE_VALUE);

        // Check the color sensor's alpha output (i.e., white value) and determine if white is present
        this.isWhite();  // Call method isWhite in "this" class to determine if color is white. Boolean variable colorWhite will be updated.

        // If color determination is made (i.e., Red, Green, Blue, or White found), return true; otherwise return valse
        return colorRed || colorGreen || colorBlue || colorWhite;
    }


    /* -------------------------------------------------------
     * Method: isWhite
     * Purpose: Determine if color is bright (i.e., white) based on the sensor's alpha output.
     *          Also update colorWhite status (true = white)
     * Return: true = bright (white)
     * ------------------------------------------------------- */
    public boolean isWhite () {
        return colorWhite = colorDetector.alpha() > MIN_WHITE_ALPHA;
    }

}     // END DeviceColorSensor
