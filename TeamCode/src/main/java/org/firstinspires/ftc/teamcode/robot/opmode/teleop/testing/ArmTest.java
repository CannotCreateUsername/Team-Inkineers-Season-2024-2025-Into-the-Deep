package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ArmTest extends LinearOpMode {
    DcMotor armMotor = null;

    public void runOpMode(){
        armMotor   = hardwareMap.get(DcMotor.class, "arm"); //the arm motor
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a){
                armMotor.setPower(0.5);
            } else if (gamepad1.b) {
                armMotor.setPower(-0.5);
            }
        }

    }
}
