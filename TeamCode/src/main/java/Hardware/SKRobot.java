/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
@Config
public class SKRobot {
    /* Public OpMode members. */
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public DcMotorEx intakeMotor;
    public DcMotor climbServo;

    public Servo droneServo;
    public Servo v4bLeft;
    public Servo v4bRight;

    public Servo leftFlapServo;
    public Servo rightFlapServo;

    public Servo rearClawServo;
    public Servo frontClawServo;
    public Servo intakePitch;

    public Servo clawRtoationServo;
    public Servo bendWristServo;

    public WebcamName webcam;
    public DistanceSensor intakeDistanceSensor;
    public DigitalChannel lredLED;
    public DigitalChannel lgreenLED;
    public DigitalChannel rredLED;
    public DigitalChannel rgreenLED;

    HardwareMap hwMap = null;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public SKRobot getInstance() {
        return this;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Reassign Hardware Map
        this.hwMap = hwMap;

        /*
         *  Drivetrain Motors
         */
        leftFront = hwMap.get(DcMotorEx.class, "parallelEncoder");
        leftBack = hwMap.get(DcMotorEx.class, "perpendicularEncoder");
        rightBack = hwMap.get(DcMotorEx.class, "rb");
        rightFront = hwMap.get(DcMotorEx.class, "rf");

        /*
         *  Subsystem Motors
         */
        climbServo = hwMap.get(DcMotor.class, "climb");
        intakeMotor = hwMap.get(DcMotorEx.class, "intake"); //intake spin
        intakePitch = hwMap.get(Servo.class, "drop"); //intake move up and down
        rearClawServo = hwMap.get(Servo.class, "rearclaw"); //tiny claw 1
        frontClawServo = hwMap.get(Servo.class, "frontclaw"); //tiny claw 2
        bendWristServo = hwMap.get(Servo.class, "bendwrist");
        clawRtoationServo = hwMap.get(Servo.class, "spinwrist");
        v4bLeft = hwMap.get(Servo.class, "laxon");
        v4bRight = hwMap.get(Servo.class, "raxon");
        v4bRight = hwMap.get(Servo.class, "raxon");
        leftFlapServo = hwMap.get(Servo.class, "lflap");
        rightFlapServo = hwMap.get(Servo.class, "rflap");
        intakeDistanceSensor = hwMap.get(DistanceSensor.class, "dispixel");
        droneServo = hwMap.get(Servo.class, "drone");

        lredLED = hwMap.get(DigitalChannel.class, "lred");
        lgreenLED = hwMap.get(DigitalChannel.class, "lgreen");
        rredLED = hwMap.get(DigitalChannel.class, "rred");
        rgreenLED= hwMap.get(DigitalChannel.class, "rgreen");
        lredLED.setMode(DigitalChannel.Mode.OUTPUT);
        lgreenLED.setMode(DigitalChannel.Mode.OUTPUT);
        rredLED.setMode(DigitalChannel.Mode.OUTPUT);
        rgreenLED.setMode(DigitalChannel.Mode.OUTPUT);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        climbServo.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        climbServo.setPower(0);
        intakeMotor.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}


