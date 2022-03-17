/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class intake {

    DcMotorEx mI;
    public static double power = 1;
    Telemetry telemetry;

    public enum IntakeState
    {
        REVERSE,
        FORWARD,
        STOP
    }

    public static IntakeState intakeState = IntakeState.STOP;
    /**
     * constructor for intake
     */
    public intake(HardwareMap hardwareMap) {
        this.mI = hardwareMap.get(DcMotorEx.class, "mI");
        this.mI.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * constructor for intake
     * @param power the power to give the motor
     */
    public intake(HardwareMap hardwareMap, double power) {
        this.power = power;
        this.mI = hardwareMap.get(DcMotorEx.class, "mI");
        this.mI.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * constructor for intake
     * @param mI the intake motor
     * @param power the power to give the motor
     * @param telemetry the telemetry object from the opmode
     */
    public intake(DcMotorEx mI, double power, Telemetry telemetry) {
        this.power = power;
        this.mI = mI;
        this.telemetry = telemetry;
        this.mI.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * set power to mI
     * setting the power given from the user to the mI
     */
    public void powerIntake(double newPower){
        mI.setPower(newPower);
    }

    /**
     * set power to mI
     * setting the power given from the constructor to the mI
     */
    public void intakeForward(){
        mI.setPower(power);
        intakeState = IntakeState.FORWARD;
    }

    /**
     * set -power to mI
     * setting the power in a negative value given from the constructor to the mI
     */
    public void intakeBackward(){
        mI.setPower(-power);
        intakeState = IntakeState.REVERSE;
    }

    /**
     * set power 0 to mI
     * stopping the mI motor
     */
    public void stop(){
        mI.setPower(0);
        intakeState = IntakeState.STOP;
    }

    /**
     * displaying intake motor power
     */
    public void displayTelemetry(){
        telemetry.addData("Intake motor power at", power);
        telemetry.update();
    }

}