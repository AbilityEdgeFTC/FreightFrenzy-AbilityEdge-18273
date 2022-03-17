package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


/*
 * Hardware class for our elevator spinner using custom aggressive pid.
 */
@Config
public class ElevatorSpinnerLibraryPID {

    public static int RIGHT_ANGLE = 225, RIGHT_ANGLE_SHARED = 150, LEFT_ANGLE = -225, LEFT_ANGLE_SHARED = -150, ZERO_ANGLE_RED = 330, ZERO_ANGLE_BLUE = -280, ZERO_ANGLE = 0;
    public static double power = 0.2;
    boolean usePID = true;
    public static double kP = 6;
    public static double kI = 0;
    public static double kD = 0;
    int target = 0;
    public static double maxPower = 0.4;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    boolean slowMove = false;
    DcMotorEx motor;
    BasicPID PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
    AngleController controller = new AngleController(PID);

    public enum SpinnerState
    {
        LEFT,
        ZERO_RED,
        ZERO_BLUE,
        ZERO_DO_NOT_USE,
        SHARED_RED,
        SHARED_BLUE,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO_DO_NOT_USE;

    Gamepad gamepad2, gamepad1;
    cGamepad cGamepad2;

    public ElevatorSpinnerLibraryPID(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.cGamepad2 = new cGamepad(gamepad2);
    }

    public ElevatorSpinnerLibraryPID(HardwareMap hardwareMap)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update()
    {
        cGamepad2.update();

        if(usePID)
        {
            switch (spinnerState)
            {
                case ZERO_DO_NOT_USE:
                    target = 0;
                    break;
                case LEFT:
                    target = LEFT_ANGLE;
                    break;
                case RIGHT:
                    target = RIGHT_ANGLE;
                    break;
                case ZERO_RED:
                    //target = ZERO_ANGLE_RED;
                    ZERO_ANGLE = ZERO_ANGLE_RED;
                    break;
                case ZERO_BLUE:
                    //target = ZERO_ANGLE_BLUE;
                    ZERO_ANGLE = ZERO_ANGLE_BLUE;
                    break;
                case SHARED_RED:
                    target = RIGHT_ANGLE_SHARED;
                    break;
                case SHARED_BLUE:
                    target = LEFT_ANGLE_SHARED;
                    break;
            }

            motor.setPower(controller.calculate(encoderTicksToRadians(target - ZERO_ANGLE), encoderTicksToRadians(motor.getCurrentPosition())));

        }
        else
        {
            if(slowMove && gamepad1.right_stick_x != 0)
            {
                motor.setPower(Range.clip(gamepad1.right_stick_x, -maxPower, maxPower));
            }
            if(gamepad2.right_stick_x != 0)
            {
                motor.setPower(Range.clip(gamepad2.right_stick_x, -maxPower, maxPower));
            }
            if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
            {
                motor.setPower(0);
            }

        }

    }

    public void updateNoAuto()
    {
        cGamepad2.update();

        if(usePID)
        {
            switch (spinnerState)
            {
                case ZERO_DO_NOT_USE:
                    target = 0;
                    break;
                case LEFT:
                    target = LEFT_ANGLE;
                    break;
                case RIGHT:
                    target = RIGHT_ANGLE;
                    break;
                case ZERO_RED:
                    target = ZERO_ANGLE_RED;
                    ZERO_ANGLE = ZERO_ANGLE_RED;
                    break;
                case ZERO_BLUE:
                    target = ZERO_ANGLE_BLUE;
                    ZERO_ANGLE = ZERO_ANGLE_BLUE;
                    break;
                case SHARED_RED:
                    target = RIGHT_ANGLE_SHARED;
                    break;
                case SHARED_BLUE:
                    target = LEFT_ANGLE_SHARED;
                    break;
            }

            motor.setPower(controller.calculate(encoderTicksToRadians(target - ZERO_ANGLE), encoderTicksToRadians(motor.getCurrentPosition())));

        }
        else
        {
            if(slowMove && gamepad1.right_stick_x != 0 && !gamepad1.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad1.right_stick_x, -maxPower, maxPower));
            }
            if(gamepad2.right_stick_x != 0 && !gamepad2.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad2.right_stick_x, -maxPower, maxPower));
            }
            if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
            {
                motor.setPower(0);
            }

            if(slowMove && gamepad1.right_stick_x != 0 && gamepad1.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad1.right_stick_x, -maxPower/2, maxPower/2));
            }
            if(gamepad2.right_stick_x != 0 && gamepad2.right_stick_button)
            {
                motor.setPower(Range.clip(gamepad2.right_stick_x, -maxPower/2, maxPower/2));
            }
            if(gamepad2.right_stick_x == 0 && gamepad1.right_stick_x == 0)
            {
                motor.setPower(0);
            }

        }

    }

    public static double encoderTicksToRadians(int ticks) {
        return Math.toRadians((ticks * 360) / TICKS_PER_REV);
    }

    public static double radiansToEncoderTicks(double radians) {
        return TICKS_PER_REV / (radians * 2 * Math.PI);
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    public boolean getUsePID()
    {
        return usePID;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }

    public static SpinnerState getSpinnerState() {
        return spinnerState;
    }

    public static void setSpinnerState(SpinnerState spinnerState) {
        ElevatorSpinnerLibraryPID.spinnerState = spinnerState;
    }

    public boolean isSlowMove() {
        return slowMove;
    }

    public void setSlowMove(boolean slowMove) {
        this.slowMove = slowMove;
    }

    public void setRIGHT_ANGLE(int RIGHT_ANGLE) {
        this.RIGHT_ANGLE = RIGHT_ANGLE;
    }

    public void setRIGHT_ANGLE_SHARED(int RIGHT_ANGLE_SHARED) {
        this.RIGHT_ANGLE_SHARED = RIGHT_ANGLE_SHARED;
    }

    public int getLEFT_ANGLE() {
        return LEFT_ANGLE;
    }

    public void setLEFT_ANGLE(int LEFT_ANGLE) {
        this.LEFT_ANGLE = LEFT_ANGLE;
    }

    public int getLEFT_ANGLE_SHARED() {
        return LEFT_ANGLE_SHARED;
    }

    public void setLEFT_ANGLE_SHARED(int LEFT_ANGLE_SHARED) {
        this.LEFT_ANGLE_SHARED = LEFT_ANGLE_SHARED;
    }

    public int getZERO_ANGLE_RED() {
        return ZERO_ANGLE_RED;
    }

    public void setZERO_ANGLE_RED(int ZERO_ANGLE_RED) {
        this.ZERO_ANGLE_RED = ZERO_ANGLE_RED;
    }

    public void setZERO_ANGLE_BLUE(int ZERO_ANGLE_BLUE) {
        this.ZERO_ANGLE_BLUE = ZERO_ANGLE_BLUE;
    }

    public int getZERO_ANGLE() {
        return ZERO_ANGLE;
    }
}