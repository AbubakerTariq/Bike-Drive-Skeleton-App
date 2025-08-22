using System;
using System.IO;
using System.Timers;
using System.Threading;
using UnityEngine;
using Articares.Distal;

public class DataManager : MonoBehaviour
{
    public static DataManager instance;

    /////////////////////////////////////////////////////////////////////////
    // Data feedback step interval ():
    /////////////////////////////////////////////////////////////////////////

    public static int DT_STEP_DATA_FBK_MSEC = 5; // CRITICAL: this must match the data feedback interval in RHB firmware
    private const int DECIM_DATA_LOG = 10;

    /////////////////////////////////////////////////////////////////////////
    // Data storage vars:
    /////////////////////////////////////////////////////////////////////////

    public string FILE_EXT = ".csv";

    public string DATA_FILE_DIR = "_data_rhb_unity_bike/";
    public string DATA_FILENAME_DEF = "data_rhb_bike_";

    // Variable to store file path of the data file
    private string dataFilePath;

    /////////////////////////////////////////////////////////////////////////
    // Time step vars:
    /////////////////////////////////////////////////////////////////////////   
    
    private int data_count = 0;

    private float t_step_prev = 0f;
    private float t_step_ref;

    /////////////////////////////////////////////////////////////////////////
    // Thread and timer to help save the data in the data file:
    /////////////////////////////////////////////////////////////////////////
    
    private System.Timers.Timer timerData;
    private Thread threadTimerData;

    private readonly object fileLock = new object();

    /////////////////////////////////////////////////////////////////////////
    // Methods:
    /////////////////////////////////////////////////////////////////////////
    ///
    private void Awake()
    {
        // Singleton logic
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
        DontDestroyOnLoad(gameObject);
    }

    private void Start()
    {
        SetupDataFile();
        SetupRecordingEvents();
    }

    private void SetupDataFile()
    {
        string dataFileName = DATA_FILENAME_DEF + DateTimeStamp() + FILE_EXT;
        dataFilePath = DATA_FILE_DIR + dataFileName;

        // The headings to be set up in the data file
        // Removed "Date Time" (13.08.2025)
        string[] headers = new[] { 
            "t sec", 
            "dt sec", 

            "pos radial",
            "dt pos radial",

            "pos rot",
            "dt pos rot",

            "pos bike x",
            "pos bike z",
            "dt pos bike x",
            "dt pos bike z",

            "pos ctrline near x",
            "pos ctrline near z",
            "vect ctrline tang x",
            "vect ctrline tang z",

            "curv ctrline near",
            "ang ctrline tang",
            "dist ctrline near",

            "bike input steer",
            "bike input throttle",
            "bike input accel",

            "bike pose ang roll",
            "bike pose ang ctrl",
            "bike pose dt_ang ctrl"
        };

        if (!File.Exists(dataFilePath))
        {
            string headerLine = string.Join(",", headers);
            File.WriteAllText(dataFilePath, $"{headerLine}\n");

            // Display section:
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("SetupDataFile(): created file [" + dataFileName + "]\n");
        }
    }

    private void SaveDataEntry(
        int count,
        DistalComm.ExerciseData distal_data,
        MotorbikeController.BikeCoords bike_coords_data,
        MotorbikeController.TrackCoords track_coords_data, 
        MotorbikeController.BikeInput bike_input_data, 
        MotorbikeController.BikePose bike_pose_data)
    {
        /////////////////////////////////////////////////////////////////////////
        // Compute time step:
        /////////////////////////////////////////////////////////////////////////
        
        const float MSEC_PER_SEC = 1000f;

        float t_step;
        float dt_step;

        if (count == 0)
        {
            t_step_ref = distal_data.UptimeMs / MSEC_PER_SEC;
            t_step = 0f;
            dt_step = 0f;
        }
        else
        {
            t_step = distal_data.UptimeMs / MSEC_PER_SEC - t_step_ref;
            dt_step = t_step - t_step_prev;
        }

        string t_step_str = t_step.ToString("F3");
        string dt_step_str = dt_step.ToString("F3");

        /////////////////////////////////////////////////////////////////////////
        // Save data step to file:
        /////////////////////////////////////////////////////////////////////////

        try
        {
            string output =
                t_step_str + "," +
                dt_step_str + "," +
                $"{distal_data.PositionR}," +
                $"{distal_data.VelocityR}," +

                $"{distal_data.PositionP}," +
                $"{distal_data.VelocityP}," +

                $"{bike_coords_data.pos_bike.x}," +
                $"{bike_coords_data.pos_bike.z}," +
                $"{bike_coords_data.dt_pos_bike.x}," +
                $"{bike_coords_data.dt_pos_bike.z}," +

                $"{track_coords_data.pos_ctrline_near.x}," +
                $"{track_coords_data.pos_ctrline_near.z}," +
                $"{track_coords_data.vect_ctrline_tang.x}," +
                $"{track_coords_data.vect_ctrline_tang.z}," +

                $"{track_coords_data.curv_ctrline_near}," +
                $"{track_coords_data.ang_ctrline_tang}," +
                $"{track_coords_data.dist_ctrline_near}," +

                $"{bike_input_data.steer}," +
                $"{bike_input_data.throttle_rhb}," +
                $"{bike_input_data.acceleration}," +

                $"{bike_pose_data.angle_roll}," +
                $"{bike_pose_data.angle_ctrl}," +
                $"{bike_pose_data.dt_angle_ctrl}";

            lock (fileLock)
            {
                File.AppendAllText(dataFilePath, $"{output}\n");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error saving data: {ex.Message}");
        }

        /////////////////////////////////////////////////////////////////////////
        // Save time step for next iteration:
        /////////////////////////////////////////////////////////////////////////

        t_step_prev = t_step;
    }

    private void SetupRecordingEvents()
    {
        ReHandyBotController.instance.OnExerciseStart += StartDataRecording;
        ReHandyBotController.instance.OnExerciseStop += StopDataRecording;
    }

    // This is for usage for SetOffsetForces command, currently being called with dummy values
    private void SetOffsetForces()
    {
        ReHandyBotController.instance.SetOffsetForces(0f, 0f);
    }

    private void Destroy()
    {
        StopDataRecording();
    }

    private void OnApplicationQuit()
    {
        StopDataRecording();
    }

    private void StartDataRecording()
    {
        StopDataRecording();

        threadTimerData = new Thread(() =>
        {
            // dataTimer = new(1f);
            timerData = new System.Timers.Timer(DT_STEP_DATA_FBK_MSEC);
            timerData.Elapsed += SaveDataOnTimerElapsed;
            timerData.AutoReset = true;
            timerData.Start();
        });
        threadTimerData.Start();
    }

    private void StopDataRecording()
    {
        threadTimerData?.Abort();
        timerData?.Stop();
        timerData?.Dispose();
    }

    private void SaveDataOnTimerElapsed(object sender, ElapsedEventArgs e)
    {
        // TODO: remove at a later date
        /*
        BikeCoords bike_coords_data = new();
        TrackCoords track_coords_data = new();
        */

        if (ReHandyBotController.instance.ExerciseActive && data_count % DECIM_DATA_LOG == 0)
        {
            // TODO: remove at a later date
            /*
            bike_coords_data.pos_bike = ReHandyBotController.instance.pos_bike;
            bike_coords_data.dt_pos_bike = ReHandyBotController.instance.dt_pos_bike;

            track_coords_data.pos_ctrline_near = ReHandyBotController.instance.pos_ctrline_near;
            track_coords_data.vect_ctrline_tang = ReHandyBotController.instance.vect_ctrline_tang;
            track_coords_data.curv_ctrline_near = ReHandyBotController.instance.curv_ctrline_near;
            track_coords_data.ang_ctrline_tang = ReHandyBotController.instance.ang_ctrline_tang;
            track_coords_data.dist_ctrline_near = ReHandyBotController.instance.dist_ctrline_near;
            */

            SaveDataEntry(
                data_count,
                ReHandyBotController.instance.distal_data,
                MotorbikeController.instance.bike_coords_data,
                MotorbikeController.instance.track_coords_data,
                MotorbikeController.instance.bike_input_data,
                MotorbikeController.instance.bike_pose_data);
        }

        data_count++;
    }

    public string DateTimeStamp()
    {
        string year = DateTime.Now.Year.ToString("0000");
        string month = DateTime.Now.Month.ToString("00");
        string date = DateTime.Now.Day.ToString("00");
        string hour = DateTime.Now.Hour.ToString("00");
        string minute = DateTime.Now.Minute.ToString("00");
        string second = DateTime.Now.Second.ToString("00");

        return year + month + date + "_" + hour + minute + second;
    }
}