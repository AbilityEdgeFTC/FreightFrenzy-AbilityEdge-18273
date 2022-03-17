/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

/**
 FOR ANY QUESTIONS PLEASE ASK ME, ELIOR YOUSEFI FROM ABILITY EDGE #18273. THIS CODE IS VERY COMPLEX AND YET, IM A LAZY
 TO ADD COMMENTS. SO FOR NOW, EVERY PART THAT YOU DONT GET, PLEASE!!!!! ASK ME.
 MY EMAIL IS: elioryous@gmail.com
 */
@Config
@TeleOp(name = "TeleOp BLUE Alliance", group = "blue")
public class teleopOwlBlue extends LinearOpMode {

    gamepad gamepad;
    ElevatorSpinnerLibraryPID spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1;
    boolean canIntake = true;
    double position = 0, positionDip = 0;
    ElapsedTime resetElevator;

    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }

    int elevatorLevel = 3;
    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad.setRedAlliance(false);
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap, gamepad1, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        resetElevator = new ElapsedTime();
        spinner.setZERO_ANGLE_BLUE(spinner.getPosition());
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.ZERO_BLUE);
        spinner.update();

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {
            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();
            elevator.update();
            spinner.update();
            carouselGamepad2();
            gamepad1And2TriggerIntake();
            elevatorSwitch();
            resetElevatorMidMoving();
            turnOnOfPidByUserAndReturnIfItWasChanged();
            if(withoutPID())
            {
                manual2ServoHandMoving();
                manual1ServoDipMoving();
            }
            gamepad2SwitchingLevels();
            telemetry.update();
        }

        gamepad.saveIMUHeading();
    }

    void carouselGamepad2()
    {
        if(gamepad2.dpad_right)
        {
            carousel.spin(false, false);
        }
        else if(gamepad2.dpad_left)
        {
            carousel.spin(true, true);
        }
        else
        {
            carousel.stop();
        }
    }

    void gamepad2SwitchingLevels()
    {
        if(gamepad2.a)
        {
            elevatorLevel = 1;
        }
        else if(gamepad2.b)
        {
            elevatorLevel = 2;
        }
        else if(gamepad2.y)
        {
            elevatorLevel = 3;
        }
        else if(gamepad2.x)
        {
            elevatorLevel = 0;
        }
    }

    void manual2ServoHandMoving()
    {
        position = hand.getPos();

        if(cGamepad2.rightBumperOnce() && hand.getPos() >= 0.03)
        {
            position -= 0.03;
            hand.moveTo(position);
        }
        else if(cGamepad2.leftBumperOnce() && hand.getPos() <= 0.97)
        {
            position += 0.03;
            hand.moveTo(position);
        }
    }

    void manual1ServoDipMoving()
    {
        positionDip = dip.getPos();

        if(cGamepad2.dpadDownOnce() && dip.getPos() >= 0.01)
        {
            positionDip -= 0.01;
            dip.moveTo(positionDip);
        }
        else if(cGamepad2.dpadUpOnce() && dip.getPos() <= 0.99)
        {
            positionDip += 0.01;
            dip.moveTo(positionDip);
        }

    }

    boolean turnOnOfPidByUserAndReturnIfItWasChanged()
    {
        if(gamepad2.left_stick_button || gamepad2.right_stick_y != 0 || gamepad2.right_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 && !gamepad2.right_stick_button)
        {
            spinner.setUsePID(false);
            elevator.setUsePID(false);
            return false;
        }
        else if((gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1) || gamepad1.left_bumper && !gamepad2.left_stick_button && elevatorMovement != ElevatorMovement.SPIN)
        {
            spinner.setUsePID(false);
            elevator.setUsePID(true);
        }

        return true;
    }

    void gamepad1And2TriggerIntake()
    {
        if ((gamepad1.right_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad1.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad2.left_trigger == 0))
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.right_trigger != 0) && canIntake && (gamepad2.left_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(-gamepad2.left_trigger);
            frontIntake = false;
            backIntake = false;
        }


        toggleIntake();
    }

    void toggleIntake()
    {
        if(frontIntake && canIntake)
        {
            intake.powerIntake(powerIntake);

        }
        else if(backIntake && canIntake)
        {
            intake.powerIntake(-powerIntake);
        }
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
        {
            intake.stop();
        }

    }

    void autoDipMoving()
    {
        if(!withoutPID())
        {
            dip.holdFreight();
        }
    }

    void manualServoMoving()
    {
        if(withoutPID())
        {
            manual2ServoHandMoving();
            manual1ServoDipMoving();
        }
    }

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                spinner.setSlowMove(false);
                manualServoMoving();
                gamepad.setCanTwist(true);

                if (gamepad1.right_bumper)
                {
                    spinner.setUsePID(false);
                    elevator.setUsePID(true);
                    spinner.setSlowMove(true);
                    gamepad.setCanTwist(false);

                    powerElevator = powerSlowElevator;
                    elevator.setPower(powerElevator);
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;

                    autoDipMoving();

                    switch (elevatorLevel)
                    {
                        case 0:
                            elevatorMovement = ElevatorMovement.SHARED;
                            break;
                        case 1:
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                    }

                    resetElevator.reset();
                }
                break;
            case LEVEL1:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                if(!withoutPID() && resetElevator.seconds() > .5)
                {
                    hand.level1();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case LEVEL2:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                if(!withoutPID() && resetElevator.seconds() > .4)
                {
                    hand.level2();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case LEVEL3:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                if(!withoutPID() && resetElevator.seconds() > .1)
                {
                    hand.level3();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case SHARED:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE);
                if(!withoutPID() && resetElevator.seconds() > .5)
                {
                    hand.shared();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case DIP:
                elevator.update();
                spinner.update();
                spinner.setUsePID(false);
                manualServoMoving();
                if(gamepad1.right_stick_button)
                {
                    gamepad.setCanTwist(true);
                    spinner.setSlowMove(false);
                }
                else
                {
                    gamepad.setCanTwist(false);
                    spinner.setSlowMove(true);
                }

                if(gamepad1.left_bumper || gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
                {
                    dip.releaseFreight();

                    switch (elevatorLevel) {
                        case 0:
                            spinner.setLEFT_ANGLE_SHARED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            elevator.setSharedHub(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case 1:
                            spinner.setLEFT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            elevator.setHubLevel1(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case 2:
                            spinner.setLEFT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            elevator.setHubLevel2(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case 3:
                            spinner.setLEFT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            elevator.setHubLevel3(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                    }

                    switch (elevatorLevel)
                    {
                        case 0:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE);
                            break;
                        case 1:
                        case 2:
                        case 3:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                            break;
                    }

                    gamepad.setCanTwist(true);
                    resetElevator.reset();

                    canIntake = true;
                    frontIntake = true;

                    if(spinner.getSpinnerState() == ElevatorSpinnerLibraryPID.SpinnerState.SHARED_BLUE)
                    {
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                        resetElevator.reset();
                    }

                    elevatorMovement = ElevatorMovement.SPIN;
                }
                break;
            default:
                elevatorMovement = ElevatorMovement.SPIN;
                break;
        }
    }

    void resetElevator()
    {
        elevator.update();
        spinner.update();

        if(!withoutPID())
        {
            dip.releaseFreight();
        }

        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        switch (elevatorLevel)
        {
            case 0:
                if(!withoutPID())
                {
                    hand.intake();
                }
                if(resetElevator.seconds() > .67 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 1:
                if(!withoutPID())
                {
                    hand.intake();
                }
                if(resetElevator.seconds() > 1.3 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 2:
                if(!withoutPID())
                {
                    hand.intake();
                }
                if(resetElevator.seconds() > 1 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 3:
                if(resetElevator.seconds() > .6 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    if(!withoutPID())
                    {
                        hand.intake();
                    }
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
        }

        elevator.update();
        spinner.update();

        canIntake = true;
    }

    void resetElevatorMidMoving()
    {
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.SPIN)
        {
            elevatorMovement = ElevatorMovement.SPIN;
            canIntake = true;
            frontIntake = true;
        }

    }

    boolean withoutPID()
    {
        if(elevator.getUsePID() == true || spinner.getUsePID() == false && elevator.getElevatorLevel() != ElevatorFirstPID.ElevatorLevel.ZERO)
        {
            switch (hand.getHandPos())
            {
                case SHARED_HUB:
                    hand.shared();
                    break;
                case INTAKE:
                    break;
                case ONE_HUB:
                    hand.level1();
                    break;
                case TWO_HUB:
                    hand.level2();
                    break;
                case THREE_HUB:
                    hand.level3();
                    break;
            }
            return false;
        }
        else
        {
            return true;
        }
    }


}