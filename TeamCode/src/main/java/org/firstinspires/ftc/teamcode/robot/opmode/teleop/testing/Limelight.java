package org.firstinspires.ftc.teamcode.robot.opmode.teleop.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@Disabled
@TeleOp(name = "Limelight", group = "Testing")
public class Limelight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        boolean selected = false;
        while (!selected) {
            if (gamepad1.a) {
                limelight.pipelineSwitch(0);
                selected = true;
            } else if (gamepad1.x) {
                limelight.pipelineSwitch(1);
                selected = true;
            } else if (gamepad1.y) {
                limelight.pipelineSwitch(2);
                selected = true;
            } else if (gamepad1.b) {
                limelight.pipelineSwitch(3);
                selected = true;
            }
            telemetry.addLine("Select:");
            telemetry.addLine("A - Yellow Sample");
            telemetry.addLine("X - Blue Bar");
            telemetry.addLine("Y - Red Bar");
            telemetry.update();
        }
        telemetry.addLine("Selected!");
        telemetry.update();
        /*
         * Starts polling for data.
         */
        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            // print some data for each detected target
            if (result.isValid()) {
                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    telemetry.addData("Area", cr.getTargetArea());
                    telemetry.addData("Weird?", cr.getTargetYPixels());
                }
            }
            telemetry.addLine("This is cringe");
            telemetry.update();
        }
    }
}