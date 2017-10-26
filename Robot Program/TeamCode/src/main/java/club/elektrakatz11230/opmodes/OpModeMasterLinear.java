package club.elektrakatz11230.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * Created by ianhaden on 8/02/2017.
 */

public abstract class OpModeMasterLinear extends LinearOpMode {


    private static OpModeMasterLinear instance = null;

    boolean initialized = false;

    boolean isInitialized() {
        return initialized;
    }

    protected void initOpenCv()
    {
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        initialized = true;
                        RobotLog.i("OpMasterLinear", "OpenCV loaded successfully");
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };


        if (!OpenCVLoader.initDebug())
        {
            RobotLog.d("OpMasterLinear", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, hardwareMap.appContext,  mLoaderCallback);
            initialized = false;
        } else
        {
            initialized = true;
            RobotLog.d("OpMasterLinear", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public OpModeMasterLinear()
    {


        super();

        instance = this;

    }


}
