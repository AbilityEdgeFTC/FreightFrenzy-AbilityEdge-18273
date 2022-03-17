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
public class hand {

    Servo sL, sR;
    Telemetry telemetry;
    public static double intakePos = 0;
    public static double level1Hub = .88, level2Hub = .775, level3Hub = .6, levelSharedHub = 1, levelDuck3 = 0.6, levelDuck2 = 0.78, levelDuck1 = 1;

    public enum HandPos
    {
        INTAKE,
        SHARED_HUB,
        ONE_HUB,
        TWO_HUB,
        THREE_HUB,
        DUCK1,
        DUCK2,
        DUCK3
    }

    public HandPos handPos = HandPos.INTAKE;

    public hand(HardwareMap hardwareMap) {
        this.sR = hardwareMap.get(Servo.class, "sRight");
        this.sL = hardwareMap.get(Servo.class, "sLeft");
        this.sR.setDirection(Servo.Direction.REVERSE);
    }

    public void intake()
    {
        handPos = HandPos.INTAKE;
        sL.setPosition(intakePos);
        sR.setPosition(intakePos);
    }

    public void shared()
    {
        handPos = HandPos.SHARED_HUB;
        sL.setPosition(levelSharedHub);
        sR.setPosition(levelSharedHub);
    }

    public void level1()
    {
        handPos = HandPos.ONE_HUB;
        sL.setPosition(level1Hub);
        sR.setPosition(level1Hub);
    }

    public void level2()
    {
        handPos = HandPos.TWO_HUB;
        sL.setPosition(level2Hub);
        sR.setPosition(level2Hub);
    }

    public void level3()
    {
        handPos = HandPos.THREE_HUB;
        sL.setPosition(level3Hub);
        sR.setPosition(level3Hub);
    }

    public void moveTo(double position)
    {
        sL.setPosition(position);
        sR.setPosition(position);
    }

    public void level3Duck()
    {
        handPos = HandPos.DUCK3;
        sL.setPosition(levelDuck3);
        sR.setPosition(levelDuck3);
    }

    public void level2Duck()
    {
        handPos = HandPos.DUCK2;
        sL.setPosition(levelDuck2);
        sR.setPosition(levelDuck2);
    }

    public void level1Duck()
    {
        handPos = HandPos.DUCK1;
        sL.setPosition(levelDuck1);
        sR.setPosition(levelDuck1);
    }

    public HandPos getHandPos() {
        return handPos;
    }

    public double getPos() {
        return sL.getPosition();
    }


    public void setHandPos(HandPos handPos) {
        this.handPos = handPos;
    }
}
