package club.elektrakatz11230.Concepts;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import club.elektrakatz11230.functions.BeaconAnalysisOCVPlayground;
import club.elektrakatz11230.functions.Constants;
import club.elektrakatz11230.functions.FileLogger;

import static com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId;

//import club.elektrakatz11230.R;


/**
 * Created by ianhaden on 4/10/2016.
 */

@Autonomous(name="Concept Vuforia Grab Image", group="5291Test")
public class ConceptVuforiaOpGrabImage extends LinearOpMode{
    OpenGLMatrix lastLocation = null;
    private double robotX;
    private double robotY;
    private double robotBearing;

    //set up the variables for the logger
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;
    public int debug;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private String allianceParkPosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;

    private static final int TARGET_WIDTH = 254;
    private static final int TARGET_HEIGHT = 184;

    @Override
    public void runOpMode() throws InterruptedException {

        //load variables
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        teamNumber = sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000");
        allianceColor = sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red");
        allianceStartPosition = sharedPreferences.getString("club.towr5291.Autonomous.StartPosition", "Left");
        allianceParkPosition = sharedPreferences.getString("club.towr5291.Autonomous.ParkPosition", "Vortex");
        delay = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0"));
        numBeacons = sharedPreferences.getString("club.towr5291.Autonomous.Beacons", "One");
        robotConfig = sharedPreferences.getString("club.towr5291.Autonomous.RobotConfig", "TileRunner-2x40");
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", null));

        debug = 3;

        //start the log
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        fileLogger = new FileLogger(runtime);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.writeEvent("init()","Log Started");

        //BeaconAnalysisOCV beaconColour = new BeaconAnalysisOCV();
        //BeaconAnalysisOCV2 beaconColour = new BeaconAnalysisOCV2();
        //BeaconAnalysisOCVAnalyse beaconColour = new BeaconAnalysisOCVAnalyse();
        BeaconAnalysisOCVPlayground beaconColour = new BeaconAnalysisOCVPlayground();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AVATY7T/////AAAAGQJxfNYzLUgGjSx0aOEU0Q0rpcfZO2h2sY1MhUZUr+Bu6RgoUMUP/nERGmD87ybv1/lM2LBFDxcBGRHkXvxtkHel4XEUCsNHFTGWYcVkMIZqctQsIrTe13MnUvSOfQj8ig7xw3iULcwDpY+xAftW61dKTJ0IAOCxx2F0QjJWqRJBxrEUR/DfQi4LyrgnciNMXCiZ8KFyBdC63XMYkQj2joTN579+2u5f8aSCe8jkAFnBLcB1slyaU9lhnlTEMcFjwrLBcWoYIFAZluvFT0LpqZRlS1/XYf45QBSJztFKHIsj1rbCgotAE36novnAQBs74ewnWsJifokJGOYWdFJveWzn3GE9OEH23Y5l7kFDu4wc";
        //parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);                                          //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1);                                                           //tells VuforiaLocalizer to only store one frame at a time
        //ConceptVuforiaGrabImage vuforia = new ConceptVuforiaGrabImage(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables velocityVortex = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheels = velocityVortex.get(0);
        wheels.setName("wheels");  // wheels target

        VuforiaTrackable tools  = velocityVortex.get(1);
        tools.setName("tools");  // tools target

        VuforiaTrackable legos = velocityVortex.get(2);
        legos.setName("legos");  // legos target

        VuforiaTrackable gears  = velocityVortex.get(3);
        gears.setName("gears");  // gears target

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(velocityVortex);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Wheels Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         *
         */

        // RED Targets
        // To Place GEARS Target
        // position is approximately - (-6feet, -1feet)

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -1 * 12 * mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        // To Place GEARS Target
        // position is approximately - (-6feet, 3feet)
        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFTCFieldWidth/2, 3 * 12 * mmPerInch, 0)
                //.translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

        //Finsih RED Targets

        // BLUE Targets
        // To Place LEGOS Target
        // position is approximately - (-3feet, 6feet)

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-3 * 12 * mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosTargetLocationOnField);
        //RobotLog.ii(TAG, "Gears Target=%s", format(legosTargetLocationOnField));

        // To Place WHEELS Target
        // position is approximately - (1feet, 6feet)
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(1 * 12 * mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsTargetLocationOnField);
        //RobotLog.ii(TAG, "Tools Target=%s", format(wheelsTargetLocationOnField));

        //Finsih BLUE Targets

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation((mmBotWidth/2), 50,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        waitForStart();

        //Mat tmp = new Mat();

        velocityVortex.activate();

        Image rgb = null;

        int loop = 0;
        Constants.BeaconColours Colour = Constants.BeaconColours.UNKNOWN;

        while (opModeIsActive()) {

            boolean gotBeacomDims = false;
            Point beaconBotRight = new Point(0,0);
            Point beaconTopLeft = new Point(0,0);
            Point beaconMiddle = new Point(0,0);

            for (VuforiaTrackable beac : velocityVortex) {

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getRawPose();

                if (pose != null) {

                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);

                    Vec2F upperLeft  = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-TARGET_WIDTH / 2,  TARGET_HEIGHT / 2, 0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F( TARGET_WIDTH / 2,  TARGET_HEIGHT / 2, 0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F( TARGET_WIDTH / 2, -TARGET_HEIGHT / 2, 0));
                    Vec2F lowerLeft  = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-TARGET_WIDTH / 2, -TARGET_HEIGHT / 2, 0));

                    double dblMidPointTopx = (upperRight.getData()[0] + upperLeft.getData()[0]) / 2;
                    double dblMidPointTopy = (upperRight.getData()[1] + upperLeft.getData()[1]) / 2;
                    double dblMidPointBotx = (lowerRight.getData()[0] + lowerLeft.getData()[0]) / 2;
                    double dblMidPointBoty = (lowerRight.getData()[1] + lowerLeft.getData()[1]) / 2;

                    double width  = Math.sqrt((Math.pow((upperRight.getData()[1] - upperLeft.getData()[1]),2)) + (Math.pow((upperRight.getData()[0] - upperLeft.getData()[0]),2)));
                    double height = Math.sqrt((Math.pow((dblMidPointTopy - dblMidPointBoty),2)) + (Math.pow((dblMidPointTopx - dblMidPointBotx),2)));

                    //width is equal to 254 mm, so width of beacon is 220mm, height of beacon is 150mm
                    //pixels per mm width, using a known size of the target
                    double dblWidthPixelsPermm = width / TARGET_WIDTH;
                    double dblHeightPixelsPermm = height / TARGET_HEIGHT;

                    //beacon base is about 25mm above top of target
                    beaconBotRight = new Point ((dblMidPointTopx + (110 * dblWidthPixelsPermm)), dblMidPointTopy - (30 * dblHeightPixelsPermm));
                    beaconTopLeft = new Point ((dblMidPointTopx - (110 * dblWidthPixelsPermm)), dblMidPointTopy - (160  * dblHeightPixelsPermm));

                    beaconMiddle.x = dblMidPointTopx;
                    beaconMiddle.y = dblMidPointTopy + (105  * dblHeightPixelsPermm);

                    gotBeacomDims = true;

                    if (debug >= 1)
                    {
                        fileLogger.writeEvent("Vuforia", "upperLeft 0 "  + upperLeft.getData()[0]);
                        fileLogger.writeEvent("Vuforia", "upperLeft 1 "  + upperLeft.getData()[1]);
                        Log.d("Vuforia", "upperLeft 0 "  + upperLeft.getData()[0]);
                        Log.d("Vuforia", "upperLeft 1 "  + upperLeft.getData()[1]);

                        fileLogger.writeEvent("Vuforia", "upperRight 0 "  + upperRight.getData()[0]);
                        fileLogger.writeEvent("Vuforia", "upperRight 1 "  + upperRight.getData()[1]);
                        Log.d("Vuforia", "upperRight 0 "  + upperRight.getData()[0]);
                        Log.d("Vuforia", "upperRight 1 "  + upperRight.getData()[1]);

                        fileLogger.writeEvent("Vuforia", "lowerLeft 0 "  + lowerLeft.getData()[0]);
                        fileLogger.writeEvent("Vuforia", "lowerLeft 1 "  + lowerLeft.getData()[1]);
                        Log.d("Vuforia", "lowerLeft 0 "  + lowerLeft.getData()[0]);
                        Log.d("Vuforia", "lowerLeft 1 "  + lowerLeft.getData()[1]);

                        fileLogger.writeEvent("Vuforia", "lowerRight 0 "  + lowerRight.getData()[0]);
                        fileLogger.writeEvent("Vuforia", "lowerRight 1 "  + lowerRight.getData()[1]);
                        Log.d("Vuforia", "lowerRight 0 "  + lowerRight.getData()[0]);
                        Log.d("Vuforia", "lowerRight 1 "  + lowerRight.getData()[1]);

                        fileLogger.writeEvent("Vuforia", "dblMidPointTopx "  + dblMidPointTopx);
                        fileLogger.writeEvent("Vuforia", "dblMidPointTopy "  + dblMidPointTopy);
                        fileLogger.writeEvent("Vuforia", "dblMidPointBotx "  + dblMidPointBotx);
                        fileLogger.writeEvent("Vuforia", "dblMidPointBoty "  + dblMidPointBoty);
                        Log.d("Vuforia", "dblMidPointTopx "  + dblMidPointTopx);
                        Log.d("Vuforia", "dblMidPointTopy "  + dblMidPointTopy);
                        Log.d("Vuforia", "dblMidPointBotx "  + dblMidPointBotx);
                        Log.d("Vuforia", "dblMidPointBoty "  + dblMidPointBoty);

                        fileLogger.writeEvent("Vuforia", "width in pixels "  + width);
                        fileLogger.writeEvent("Vuforia", "height in pixels "  + height);
                        Log.d("Vuforia", "width in pixels "  + width);
                        Log.d("Vuforia", "height in pixels "  + height);
                    }
                }
            }

            if (gotBeacomDims) {

                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

                long numImages = frame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgb = frame.getImage(i);
                        break;
                    }
                }

            /*rgb is now the Image object that we’ve used in the video*/
                Log.d("OPENCV", "Height " + rgb.getHeight() + " Width " + rgb.getWidth());

                Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(rgb.getPixels());
                Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
                Utils.bitmapToMat(bm, tmp);

                if (beaconTopLeft.x < 0)
                    beaconTopLeft.x = 0;
                if (beaconTopLeft.y < 0)
                    beaconTopLeft.y = 0;
                if (beaconBotRight.x > rgb.getWidth())
                    beaconBotRight.x = rgb.getWidth();
                if (beaconBotRight.y > rgb.getHeight())
                    beaconBotRight.y = rgb.getHeight();

                frame.close();
                //Constants.BeaconColours Colour = beaconColour.beaconAnalysisOCV(tmp, loop);
                //Constants.BeaconColours Colour = beaconColour.beaconAnalysisOCV2(tmp, loop, debug);
                Colour = beaconColour.BeaconAnalysisOCVPlayground(debug, tmp, loop, beaconTopLeft, beaconBotRight, beaconMiddle);
                loop++;
                Log.d("OPENCV", "Returned " + Colour);
            }

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                // Then you can extract the positions and angles using the getTranslation and getOrientation methods.
                VectorF trans = lastLocation.getTranslation();
                Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in Cartesian system) position is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;
                if (robotBearing < 0)
                {
                    robotBearing = 360 + robotBearing;
                }

                telemetry.addData("Pos X ", robotX);
                telemetry.addData("Pos Y ", robotY);
                telemetry.addData("Bear  ", robotBearing);
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos   ", format(lastLocation));

                telemetry.addData("Text ", "*** Vision Data***");
                //telemetry.addData("Red  ", "Red :  " + redpoint);
                //telemetry.addData("Blue ", "Blue:  " + bluepoint);
                //telemetry.addData("Dir  ", "Direction:  " + directionOfBeacon);
            } else {
                telemetry.addData("Pos   ", "Unknown");
            }

            switch (Colour) {
                case BEACON_BLUE_RED:
                    telemetry.addData("Beacon ", "Blue Red");
                    break;
                case BEACON_RED_BLUE:
                    telemetry.addData("Beacon ", "Red Blue");
                    break;
                case BEACON_BLUE_LEFT:
                    telemetry.addData("Beacon ", "Blue Left");
                    break;
                case BEACON_RED_LEFT:
                    telemetry.addData("Beacon ", "Red Left");
                    break;
                case BEACON_BLUE_RIGHT:
                    telemetry.addData("Beacon ", "Blue Right");
                    break;
                case BEACON_RED_RIGHT:
                    telemetry.addData("Beacon ", "Red Right");
                    break;
                case UNKNOWN:
                    telemetry.addData("Beacon ", "Unknown");
                    break;
            }


            telemetry.update();

        }

        //stop the log
        if (fileLogger != null)
        {
            fileLogger.writeEvent("stop()","Stopped");
            fileLogger.close();
            fileLogger = null;
        }

    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
