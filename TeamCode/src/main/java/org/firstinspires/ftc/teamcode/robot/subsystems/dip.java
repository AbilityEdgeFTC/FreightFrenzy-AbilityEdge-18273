/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class dip {

    Servo sD;
    public static double releasingPosition = .56, holdingPosition = .04;
    Telemetry telemetry;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public dip(HardwareMap hardwareMap) {
        this.sD = hardwareMap.get(Servo.class, "sD");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public dip(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sD = hardwareMap.get(Servo.class, "sD");
        this.telemetry = telemetry;
    }

    // move dip servo to intake position.
    public void getFreight() {
        sD.setPosition(releasingPosition);
    }

    // move dip servo to releasing position.
    public void releaseFreight() {
        sD.setPosition(releasingPosition);
    }

    // move dip servo to holding position.
    public void holdFreight() {
        sD.setPosition(holdingPosition);
    }

    // get dip servo position
    public double getPos() {
        return sD.getPosition();
    }

    // move to custom position
    public void moveTo(double position)
    {
        sD.setPosition(position);
    }
}
