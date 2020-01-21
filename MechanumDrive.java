/*
Copyright 2019 FIRST Tech Challenge Team 14853

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,http://192.168.49.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/MechanumDrive.java
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Comparator;
import java.util.stream.Stream;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class MechanumDrive extends LinearOpMode {
    private Blinker expansion_Hub_2;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor armMotor;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private Servo miniArm;
    private Servo frontServo;
    private Servo backServo;
    private BNO055IMU imu;

    private final double sensitivity = 1;
    
    private double intakePower;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        frontServo = hardwareMap.get(Servo.class, "frontServo");
        backServo = hardwareMap.get(Servo.class, "backServo");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        miniArm = hardwareMap.get(Servo.class, "Mini Arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int armPosition = armMotor.getCurrentPosition();
        
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        imu.initialize(parameters);
        
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        
            Orientation angles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);
            
            double forward = -gamepad1.left_stick_y;
            double right  =  -gamepad1.left_stick_x;
            double clockwise = gamepad1.right_stick_x;
            clockwise *= sensitivity;
            
        
            //this part of the code controls the mechanum drive
            //double theta = angles.firstAngle * Math.PI/180;
            double theta = 0;
            
            
            double temp = forward * Math.cos(theta) + right * Math.sin(theta);
            right = forward * Math.sin(theta) - right * Math.cos(theta);
            forward = temp;
            right = -right;
            
            double frontLeftPower = -forward - clockwise - right;
            double frontRightPower = forward - clockwise - right;
            double rearLeftPower = -forward - clockwise + right;
            double rearRightPower = forward - clockwise + right;
            
            
            double max = Math.abs(frontLeftPower);
            if (Math.abs(frontRightPower) > max) {
                max=Math.abs(frontRightPower);
            } 
            if (Math.abs(rearRightPower) > max) {
                max=Math.abs(rearRightPower);
            } 
            if (Math.abs(rearLeftPower) > max) {
                max=Math.abs(rearLeftPower);
            } 
            
            if(max>1){
                frontLeftPower /= max;
                frontRightPower /= max;
                rearLeftPower /= max;
                rearRightPower /= max;
            }
            backLeft.setPower(rearLeftPower);
            backRight.setPower(rearRightPower);
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            
            // this part of the code controls the arm.
            // we subtract the left trigger from the right because
            // we want to have the right trigger go backwards.
            // the gamepad.left-gamepad.right_trigger float is being multiplied by 25, 
            // the power we want the motor to be at full depression. 
            armPosition+=25*(gamepad2.left_trigger-gamepad2.right_trigger);
            // we add gamepad.left_trigger to gamepad.right_trigger.
            armMotor.setPower(gamepad2.left_trigger+gamepad2.right_trigger);
            
            
            telemetry.addData("armPosition: ", armPosition);
            armMotor.setTargetPosition(armPosition);
            
            // This is a ternary operator, which is 
            // a statment true/false input that returns
            // a value based on the true/false input.
            // In this case, if you hit a, we return
            // 1. If not, if b is pressed return -1,
            // otherwise, return 0.
            intakePower = gamepad2.a ? 1.0 : gamepad2.b ? -1.0 : 0.0;
    
            intakeLeft.setPower(-intakePower);
            intakeRight.setPower(intakePower);
            
            if (gamepad2.x){
                miniArm.setPosition(0.7);
            } else if (gamepad2.y){
                miniArm.setPosition(0.1);
            }
            if(gamepad2.right_bumper) {
                frontServo.setPosition(.3);
                backServo.setPosition(.05);
            } else if (gamepad2.left_bumper) {
                frontServo.setPosition(.2);
                backServo.setPosition(.15);
            }
            
            telemetry.addData("Status", "Running");
            telemetry.addData("angle", angles.firstAngle);
            telemetry.update();
    
        }
    }
}
