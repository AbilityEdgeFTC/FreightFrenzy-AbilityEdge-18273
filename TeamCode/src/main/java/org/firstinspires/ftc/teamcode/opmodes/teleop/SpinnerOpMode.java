package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@TeleOp(name = "Elevator Spinner Testing", group = "testing")
@Disabled
public class SpinnerOpMode extends LinearOpMode {

    SpinnerFirstPID elevator;

    public static boolean usePID = false;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new SpinnerFirstPID(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            if(usePID)
            {
                elevator.update();
            }
            else
            {
                elevator.setUsePID(false);
            }

            telemetry.addData("POS", elevator.getPosition());
            telemetry.addData("TARGET", elevator.getTarget());
            telemetry.update();
        }
    }
}
