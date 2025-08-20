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
    // Data structures:
    /////////////////////////////////////////////////////////////////////////
    private struct BikeData
    {
        public Vector3 pos_bike;
        public Vector3 vect_dir_bike;
        public Vector3 dt_pos_bike;
    }

    private struct TrackData
    {
        public Vector3 pos_ctrline_near;
        public Vector3 vect_ctrline_tang;
        public float curv_ctrline_near;
        public float ang_ctrline_tang;
        public float dist_ctrline_near;
    }

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
            "vect dir bike x",
            "vect dir bike z",
            "dt pos bike x",
            "dt pos bike z",

            "pos ctrline near x",
            "pos ctrline near z",
            "vect ctrline tang x",
            "vect ctrline tang z",

            "curv ctrline near",
            "ang ctrline tang",
            "dist ctrline near"
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

    private void SaveDataEntry(DistalComm.ExerciseData distal_data, BikeData bike_data, TrackData track_data)
    {
        // string datetime = DateTime.UtcNow.ToLocalTime().ToString("MMM-dd-yyyy HH:mm:ss.fff tt \"GMT\"zzz");

        /////////////////////////////////////////////////////////////////////////
        // Compute time step:
        /////////////////////////////////////////////////////////////////////////
        ///
        const float MSEC_PER_SEC = 1000f;

        float t_step;
        float dt_step;

        if (data_count == 0)
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

                $"{bike_data.pos_bike.x}," +
                $"{bike_data.pos_bike.z}," +
                $"{bike_data.vect_dir_bike.x}," +
                $"{bike_data.vect_dir_bike.z}," +
                $"{bike_data.dt_pos_bike.x}," +
                $"{bike_data.dt_pos_bike.z}," +

                $"{track_data.pos_ctrline_near.x}," +
                $"{track_data.pos_ctrline_near.z}," +
                $"{track_data.vect_ctrline_tang.x}," +
                $"{track_data.vect_ctrline_tang.z}," +

                $"{track_data.curv_ctrline_near}," +
                $"{track_data.ang_ctrline_tang}," +
                $"{track_data.dist_ctrline_near},";

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
        BikeData bike_data;
        TrackData track_data;

        // Modified counter (13.08.2025):
        if (data_count % DECIM_DATA_LOG == 0)
        {
            bike_data.pos_bike = ReHandyBotController.instance.pos_bike;
            bike_data.vect_dir_bike = ReHandyBotController.instance.vect_dir_bike;
            bike_data.dt_pos_bike = ReHandyBotController.instance.dt_pos_bike;

            track_data.pos_ctrline_near = ReHandyBotController.instance.pos_ctrline_near;
            track_data.vect_ctrline_tang = ReHandyBotController.instance.vect_ctrline_tang;
            track_data.curv_ctrline_near = ReHandyBotController.instance.curv_ctrline_near;
            track_data.ang_ctrline_tang = ReHandyBotController.instance.ang_ctrline_tang;
            track_data.dist_ctrline_near = ReHandyBotController.instance.dist_ctrline_near;

            SaveDataEntry(ReHandyBotController.instance.DistalData, bike_data, track_data);
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