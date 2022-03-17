/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class carousel {

    //2 servos for carousel
    CRServo sCL, sCR;
    Telemetry telemetry;
    public static double powerCarousel = 1;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carousel(HardwareMap hardwareMap) {
        this.sCL = hardwareMap.get(CRServo.class, "sCL");
        this.sCR = hardwareMap.get(CRServo.class, "sCR");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sCL = hardwareMap.get(CRServo.class, "sCL");
        this.sCR = hardwareMap.get(CRServo.class, "sCR");
        this.telemetry = telemetry;
    }

    // spin carousel servos with power, option for reversed is added/red alliance is there to 2 which side should the
    // servos spin, and which servos.
    public void spin(boolean reverse, boolean isRed){
        if (isRed) {
            if (reverse)
            {
                sCL.setPower(powerCarousel);
            }
            else
            {
                sCL.setPower(-powerCarousel);
            }
        }
        else
        {
            if (reverse) {
                sCR.setPower(-powerCarousel);
            } else {
                sCR.setPower(powerCarousel);
            }
        }
    }

    // stop both of the servos.
    public void stop(){
        sCL.setPower(0);
        sCR.setPower(0);
    }
}
