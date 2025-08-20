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

        /*
        pos_bike
        vect_dir_bike
        dt_pos_bike

        pos_ctrline_near
        vect_ctrline_tang
        curv_ctrline_near
        ang_ctrline_tang
        dist_ctrline_near
        */

        // The headings to be set up in the data file
        // Removed "Date Time" (13.08.2025)
        string[] headers = new[] { 
            "t sec", 
            "dt sec", 
            "pos radial", 
            "vel radial", 
            "pos rot", 
            "vel rot", 
            "pos x", 
            "pos z", 
            "angle dir", 
            "vel magn", 
            "angle tilt",
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
        // Modified counter (13.08.2025):
        if (data_count % DECIM_DATA_LOG == 0)
        {
            SaveDataEntry(ReHandyBotController.instance.DistalData, 0f, 0f, 0f, 0f, 0f);
        }

        data_count++;
    }

    private void SaveDataEntry( DistalComm.ExerciseData DistalData, 
        float cartesianPositionX, float cartesianPositionZ, float directionAngle, float speed, float tiltAngle)
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
            t_step_ref = DistalData.UptimeMs / MSEC_PER_SEC;
            t_step = 0f;
            dt_step = 0f;
        }
        else
        {
            t_step = DistalData.UptimeMs / MSEC_PER_SEC - t_step_ref;
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
                $"{DistalData.PositionR}," +
                $"{DistalData.VelocityR}," +
                $"{DistalData.PositionP}," +
                $"{DistalData.VelocityP}," +
                $"{cartesianPositionX}," +
                $"{cartesianPositionZ}," +
                $"{directionAngle}," +
                $"{speed}," +
                $"{tiltAngle}";
                // $"{curv_ctrline_near}," +
                // $"{ang_ctrline_tang}," +
                // $"{dist_ctrline_near},";
               
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