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

    public const int DT_STEP_DATA_FBK_MSEC = 5; // CRITICAL: this must match the data feedback interval in RHB firmware
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
    
    private int data_recv_count = 0;

    private float t_step_prev = 0f;
    private float t_step_ref = 0f;

    public bool isRaceStarted = false; // added to avoid ambiguous 'active exercise' situations during data logging (27.08.2025)

    /////////////////////////////////////////////////////////////////////////
    // Thread and timer to help save the data in the data file:
    /////////////////////////////////////////////////////////////////////////

    private System.Timers.Timer timerData;
    private Thread threadTimerData;

    private readonly object fileLock = new object();

    /////////////////////////////////////////////////////////////////////////
    // Methods:
    /////////////////////////////////////////////////////////////////////////
    
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

    }

    public void SetupDataFile()
    {
        string dataFileName = DATA_FILENAME_DEF + DateTimeStamp() + FILE_EXT;
        dataFilePath = DATA_FILE_DIR + dataFileName;

        // Headings to be set up in the data file
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
            "dir bike x",
            "dir bike z",

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
            "bike pose dt ang roll",
            // "bike pose ang ctrl",
            // "bike pose dt ang ctrl",
            "bike pose ang steer wheel",

            "pos preview x",
            "pos preview z",
            "pos track targ x",
            "pos track targ z",
            "angle roll targ",
            "dt angle roll targ",
            "input steer targ",
            "curv ctrline targ",
            "sin dev targ",
            "vect ctrline tangent targ x",
            "vect ctrline tangent targ z",

            /*
            "steer update 0",
            "steer update 1",
            "steer update 2",
            "steer update 3",
            */

            "factor steer bike speed",
            "steer term input",
            "steer term angle ctrl",
            "steer term dt angle ctrl"
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
        float t, float dt, 
        DistalComm.ExerciseData distal_data,
        MotorbikeController.BikeCoords bike_coords_data,
        MotorbikeController.TrackCoords track_coords_data, 
        MotorbikeController.BikeInput bike_input_data, 
        MotorbikeController.BikePose bike_pose_data,
        MotorbikeController.FeedbackControl fbk_ctrl_data,
        MotorbikeController.SteerCalc steer_calc_data)
    {
        string t_step_str = t.ToString("F3");
        string dt_step_str = dt.ToString("F3");

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
                $"{bike_coords_data.dir_unit_bike.x}," +
                $"{bike_coords_data.dir_unit_bike.z}," +

                $"{track_coords_data.pos_ctrline_near.x}," +
                $"{track_coords_data.pos_ctrline_near.z}," +
                $"{track_coords_data.vect_ctrline_tang.x}," +
                $"{track_coords_data.vect_ctrline_tang.z}," +

                $"{track_coords_data.curv_ctrline_near}," +
                $"{track_coords_data.ang_ctrline_tang}," +
                $"{track_coords_data.dist_ctrline_near}," +

                $"{bike_input_data.steer_scaled}," +
                $"{bike_input_data.throttle}," +
                $"{bike_input_data.acceleration}," +

                $"{bike_pose_data.angle_roll_bike}," +
                $"{bike_pose_data.dt_angle_roll_bike}," +
                // $"{bike_pose_data.angle_ctrl}," +
                // $"{bike_pose_data.dt_angle_ctrl}," +
                $"{bike_pose_data.angle_steer_wheel_fwd}," +

                $"{fbk_ctrl_data.pos_preview.x}," +
                $"{fbk_ctrl_data.pos_preview.z}," +
                $"{fbk_ctrl_data.pos_track_targ.x}," +
                $"{fbk_ctrl_data.pos_track_targ.z}," +
                $"{fbk_ctrl_data.angle_roll_targ}," +
                $"{fbk_ctrl_data.dt_angle_roll_targ}," +
                $"{fbk_ctrl_data.input_steer_targ}," +
                $"{fbk_ctrl_data.curv_ctrline_targ}," +
                $"{fbk_ctrl_data.sin_dev_targ}," +
                $"{fbk_ctrl_data.vect_ctrline_tang_target.x}," +
                $"{fbk_ctrl_data.vect_ctrline_tang_target.z}," +
                
                /*
                $"{steer_calc_data.steer_update[0]}," +   
                $"{steer_calc_data.steer_update[1]}," +   
                $"{steer_calc_data.steer_update[2]}," +   
                $"{steer_calc_data.steer_update[3]}," +   
                */

                $"{steer_calc_data.factor_steer_bike_speed}," +
                $"{steer_calc_data.steer_term_input}," +
                $"{steer_calc_data.steer_term_angle_ctrl}," +
                $"{steer_calc_data.steer_term_dt_angle_ctrl}" ;
                

            lock (fileLock)
            {
                File.AppendAllText(dataFilePath, $"{output}\n");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error saving data: {ex.Message}");
        }
    }

    public void SetupRecordingEvents()
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
        // Launch stop data log thread:
        StopDataRecording();

        // Create new CSV file:
        SetupDataFile();

        // 'Race started' flag:
        isRaceStarted = true;

        // Reset data received counter:
        data_recv_count = 0;

        // Set up timer:
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
        const float MSEC_PER_SEC = 1000f;
        float DT_STEP_DATA_LOG   =  DECIM_DATA_LOG * DT_STEP_DATA_FBK_MSEC / MSEC_PER_SEC;

        const int N_DATA_REC_COUNTS_DROP = 3*DECIM_DATA_LOG; // number of initial counts to drop (to avoid initial timing bugs)

        float t_step;
        float dt_step;
        float uptime_msec;

        if (isRaceStarted)
        {
            int data_recv_count_offs = data_recv_count - N_DATA_REC_COUNTS_DROP;

            if (data_recv_count_offs >= 0 && data_recv_count_offs % DECIM_DATA_LOG == 0)
            {

                /////////////////////////////////////////////////////////////////////////
                // Compute time step:
                /////////////////////////////////////////////////////////////////////////

                uptime_msec = ReHandyBotController.instance.distal_data.UptimeMs;

                if (data_recv_count_offs == 0) { 
                    t_step_ref = uptime_msec / MSEC_PER_SEC;

                    t_step  = 0f;
                    dt_step = 0f;
                }
                else
                {
                    t_step  = uptime_msec / MSEC_PER_SEC - t_step_ref;
                    dt_step = t_step - t_step_prev;
                }                 
                    
                /////////////////////////////////////////////////////////////////////////
                // Save data entry:
                /////////////////////////////////////////////////////////////////////////

                if (data_recv_count_offs == 0 || dt_step > DT_STEP_DATA_LOG / 2f) // condition aims to prevent duplicate entries
                    SaveDataEntry(
                        t_step, 
                        dt_step,
                        ReHandyBotController.instance.distal_data,
                        MotorbikeController.instance.bike_coords_data,
                        MotorbikeController.instance.track_coords_data,
                        MotorbikeController.instance.bike_input_data,
                        MotorbikeController.instance.bike_pose_data,
                        MotorbikeController.instance.fbk_ctrl_data,
                        MotorbikeController.instance.steer_calc_data);

                /////////////////////////////////////////////////////////////////////////
                // Save time step for next iteration:
                /////////////////////////////////////////////////////////////////////////

                t_step_prev = t_step;
            }

            data_recv_count++;
        }  
    }

    public string DateTimeStamp()
    {
        string year   = DateTime.Now.Year.ToString("0000");
        string month  = DateTime.Now.Month.ToString("00");
        string date   = DateTime.Now.Day.ToString("00");
        string hour   = DateTime.Now.Hour.ToString("00");
        string minute = DateTime.Now.Minute.ToString("00");
        string second = DateTime.Now.Second.ToString("00");

        return year + month + date + "_" + hour + minute + second;
    }
}