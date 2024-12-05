package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//@Disabled
@Autonomous(name = "MI States Auton 23870", group = "final")
public class RoboAvengersMIStatesAuton extends OpMode
{
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  sliderMotor      = null;
    public Servo    claw             = null; //the claw servo
    public Servo    clawHead         = null;

    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_CLEAR_BARRIER         = 25 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET     = 100 * ARM_TICKS_PER_DEGREE;
    final double SLIDER_TICKS_PER_MM = 537.7 / 120.0;
    final double SLIDER_SCORING_IN_HIGH_BASKET = 465 * SLIDER_TICKS_PER_MM;
    final double SLIDER_SAMPLE1_PICKUP = 200 * SLIDER_TICKS_PER_MM;
    final double SLIDER_SAMPLE1_SCORE = 220 * SLIDER_TICKS_PER_MM;
    final double CLAW_CLOSED = 0.0;
    final double CLAW_OPEN   = 1.0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; //variable to store the state of our auto
    private PathChain scorePreload, pickSample1Path, scoreSample1path;
    
    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * [RAC] Our robot is 15.25 by 17.5 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */
    
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.75, 103.625, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose1 = new Pose(8.75, 130, Math.toRadians(90));

    // Back away from bucket after scorePose1
    //private final Pose backup1Pose = new Pose (8.75, 120, Math.toRadians(90));

    // Pickup pose of our robot to collect the first sample (right)
    private final Pose pickSample1Pose = new Pose (24,112, Math.toRadians(45));

    // Back away from bucket after scorePose2
    private final Pose scorePose2 = new Pose (12, 132, Math.toRadians(135));


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init()
    {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initiate drive motors
        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.3);
        follower.setStartingPose(startPose);

        // Initiate arm motor and claw servo
        armMotor        = hardwareMap.dcMotor.get("left_arm");
        sliderMotor     = hardwareMap.dcMotor.get("liftMotor");
        claw            = hardwareMap.servo.get("claw");
        clawHead        = hardwareMap.servo.get("clawHead");

        //define behavior of motors
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //define behavior of motors
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setTargetPosition(0);

        // Build robot auton paths according to Pedro Pathing coordinates
        buildPaths();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop()
    {
    }

    /** This builds the path required for our robot to move and collect samples **/
    private void buildPaths()
    {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        pickSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickSample1Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickSample1Pose.getHeading())
                .build();

        scoreSample1path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSample1Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickSample1Pose.getHeading(), scorePose2.getHeading())
                .build();
    }

    /** Runs the auton path **/
    private void autonomousPathUpdate()
    {
        {
            switch (pathState)
            {
                case 0:
                    actionTimer.resetTimer();
                    claw.setPosition(CLAW_CLOSED);
                    while (actionTimer.getElapsedTime() < 500)
                    {
                        telemetry.addData("Case 0: Claw closed", claw.getPosition());
                        telemetry.update();
                    }
                    setPathState(1);
                    break;

                case 1:
                    armMotor.setTargetPosition((int) ARM_SCORE_HIGH_BASKET);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.4);

                    sliderMotor.setTargetPosition((int) SLIDER_SCORING_IN_HIGH_BASKET);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setPower(0.4);

                    while (armMotor.isBusy() || sliderMotor.isBusy() )
                    {
                        telemetry.addData("Case 1: Robot arm ready for the top scoring basket: ", "Complete");
                        telemetry.update();
                    }
                    setPathState(2);
                    break;

                case 2:
                    //this holds the robot in position
                    follower.followPath(scorePreload, true);
                    telemetry.addData("Case 2: Robot current x position", follower.getPose().getX());
                    telemetry.addData("Case 2: Robot current y position", follower.getPose().getY());
                    telemetry.addData("Case 2: Robot current heading", follower.getPose().getHeading());
                    setPathState(3);
                    break;

                case 3:
                    // Step 4 Sample drop in top basket
                    if(follower.isBusy() == false && follower.getPose().getX() > (scorePose1.getX() - 1.0) && follower.getPose().getY() > (scorePose1.getY() - 1.0))
                    {
                        actionTimer.resetTimer();
                        clawHead.setPosition(0.7);
                        telemetry.addData("Case 3: Claw rotated", "completed");
                        telemetry.update();
                        claw.setPosition(CLAW_OPEN); //[TBT] Moved outside the while loop

                        while (actionTimer.getElapsedTime() < 500) //increase it to 500 if needed
                        {
                            telemetry.addData("Case 3: Sample dropped: ", "Complete");
                        }
                        setPathState(4);
                    }
                    break;

                case 4:
                    // Step 5 Strafe backwards to sample 1 pickup
                    //this holds the robot in position
                    if(follower.isBusy() == false && follower.getPose().getX() > (scorePose1.getX() - 1.0) && follower.getPose().getY() > (scorePose1.getY() - 1.0))
                    {
                        follower.followPath(pickSample1Path, true);
                        telemetry.addData("Case 4: Robot current x position", follower.getPose().getX());
                        telemetry.addData("Case 4: Robot current y position", follower.getPose().getY());
                        telemetry.addData("Case 4: Robot current heading", follower.getPose().getHeading());
                        setPathState(5);
                    }
                    break;

                case 5:
                    //Step 5 Retract arm for sample 1 collection
                    if(follower.isBusy() == false && follower.getPose().getX() < (pickSample1Pose.getX() + 1.0) && follower.getPose().getY() < (pickSample1Pose.getY() + 1.0))
                    {
                        actionTimer.resetTimer();
                        clawHead.setPosition(0.25);
                        while (actionTimer.getElapsedTime() < 250)
                        {
                            telemetry.addData("Case 5: Claw head rotated 0.25", "completed");
                            telemetry.update();
                        }

                        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                        sliderMotor.setTargetPosition((int)SLIDER_SAMPLE1_PICKUP);
                        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sliderMotor.setPower(0.3);

                        while ( sliderMotor.isBusy() )
                        {
                            telemetry.addData("Case 5: Robot arm retract position", "InProgress");
                            telemetry.update();
                        }

                        armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(0.3);

                        while (armMotor.isBusy() )
                        {
                            telemetry.addData("Case 5: Robot arm sample pickup position", "InProgress");
                            telemetry.update();
                        }
                        setPathState(6);
                    }
                    break;

                case 6:
                    actionTimer.resetTimer();
                    claw.setPosition(CLAW_CLOSED);
                    while (actionTimer.getElapsedTime() < 500)
                    {
                        telemetry.addData("Case 6: Claw closed", claw.getPosition());
                        telemetry.update();
                    }
                    setPathState(7);
                    break;

                case 7:
                    armMotor.setTargetPosition((int) ARM_SCORE_HIGH_BASKET);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.3);

                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setTargetPosition((int) SLIDER_SAMPLE1_SCORE);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setPower(0.3);

                    while (armMotor.isBusy() || sliderMotor.isBusy() )
                    {
                        telemetry.addData("Case 7: Robot arm ready for the top scoring basket: ", "Complete");
                        telemetry.update();
                    }
                    setPathState(8);
                    break;

                case 8:
                    //Step 6 Go to sample 1 position
                    if(follower.isBusy() == false && follower.getPose().getX() > (pickSample1Pose.getX() - 1.0) && follower.getPose().getY() > (pickSample1Pose.getY() - 1.0))
                    {
                        follower.followPath(scoreSample1path, true);
                        telemetry.addData("Case 8: Robot current x position", follower.getPose().getX());
                        telemetry.addData("Case 8: Robot current y position", follower.getPose().getY());
                        telemetry.addData("Case 8: Robot current heading", follower.getPose().getHeading());
                        setPathState(9);
                    }
                    break;
                case 9:
                    // Step 4 Sample drop in top basket
                    if(follower.isBusy() == false && follower.getPose().getX() > (scorePose2.getX() - 1.0) && follower.getPose().getY() > (scorePose2.getY() - 1.0))
                    {
                        actionTimer.resetTimer();
                        clawHead.setPosition(0.7);
                        telemetry.addData("Case 9: Claw rotated 0.7", "completed");
                        telemetry.update();
                        claw.setPosition(CLAW_OPEN); //[TBT] Moved outside the while loop

                        while (actionTimer.getElapsedTime() < 500) //increase it to 500 if needed
                        {
                            telemetry.addData("Case 3: Sample dropped: ", "Complete");
                        }
                        setPathState(4);
                    }
                    break;
            }
        }
    }

    /** Identifies the current state of the path **/
    private void setPathState(int pState)
    {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
