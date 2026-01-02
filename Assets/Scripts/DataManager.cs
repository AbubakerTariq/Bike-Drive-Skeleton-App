using System;
using System.IO;
using System.Timers;
using System.Threading;
using UnityEngine;
using Articares.Distal;
using PimDeWitte.UnityMainThreadDispatcher;

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

    private string FILE_EXT = ".csv";

    // DATA file name identifiers:
    private string DATA_FILE_DIR = "C:/_data_rhb_unity_bike/";
    private string DATA_FILENAME_DEF = "data_rhb_bike_";

    // DATA file: variable to store file path of the 
    private string dataFilePath;

    // PARAMETER file name identifiers:
    private string PARAM_FILENAME_DEF = "param_rhb_bike_";

    // PARAMETER file: variable to store file path of the 
    private string paramFilePath;

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

    public void SetupDataFile(string dataFilePathThis)
    {
        // Headings for the data file:
        string[] headers = new[] { 
            "t sec", 
            "dt sec", 

            "pos radial",
            "dt pos radial",
            "pos rot",
            "dt pos rot",
            "pos rot eq ref",
            "torque assist",

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
            "ang ctrline tang",
            "dist ctrline near",

            "bike input steer",
            "bike input throttle",
            "bike input accel",

            "ang roll bike",
            "dt ang roll bike",
            "ang steer wheel bike",

            "pos preview x",
            "pos preview z",
            "pos track targ x",
            "pos track targ z",
            "angle roll targ",
            "dt angle roll targ",
            "input steer targ",
            "sin dev targ",
            "vect ctrline tangent targ x",
            "vect ctrline tangent targ z",
            "err pos preview targ x",
            "err pos preview targ z",
            "err pos preview targ val",
            "curv track targ",
            "angle roll gain lo",
            "angle roll gain hi",

            // "steer update 0",
            // "steer update 1",
            // "steer update 2",
            // "steer update 3",
            // "factor steer bike speed",
            // "steer term input",
            // "steer term angle ctrl",
            // "steer term dt angle ctrl",

            "Force X L",
            "Force Y L",
            "Force X R",
            "Force Y R",

            "angle roll steer equiv",
            "step count underst",
            "step count moving",
            "step count",
            "dist traveled"
        };

        if (!File.Exists(dataFilePathThis))
        {
            string headerLine = string.Join(",", headers);
            File.WriteAllText(dataFilePathThis, $"{headerLine}\n");

            // Display section:
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("SetupDataFile(): created DATA file [" + dataFilePathThis + "]\n");
        }
    }

    public void SetupParametersFile(string paramFilePathThis)
    {
        // Headings for the data file:
        string[] headers = new[] {
            "DT_STEP_APP_MSEC",
            "USE_BEGINNER_BIKE_CONSTR",
            "CASE_CTRL_MODE",
            "POS_RADIAL_THROT_ZERO",
            "K_STIFF_RADIAL_THROT_MANUAL",
            "SPEED_AUTO_THROTTLE_MAX_KPH",
            "FORCE_GAIN_RADIAL",
            "FORCE_GAIN_ROT",
            "K_STIFF_RADIAL_THROT_AUTO",
            "K_STIFF_ROT_BASE",
            "K_STIFF_ROT_TRACKING",
            "B_DAMP_ROT_TRACKING",
            "FRAC_ASSIST_STIFF",
            "TORQUE_ASSIST_STEER_MAX",
            "FACT_ASSIST_STEER",
            "GAME_LEVEL_MID",
            "FACT_ASSIST_MID",
            "FRAC_POS_ROT_INPUT_PATIENT",
            "FACT_ASSIST_THROTTLE",
 
            "DT_PREVIEW",
            "P_GAIN_ASSIST",
            "P_GAIN_TRACK",
            "P_GAIN_LO",
            "FACT_ASSIST_STEER_MAX",
            "OFFS_FACT_ASSIST_STEER",
            "TORQUE_MOTOR_MAX",
            "FACTOR_ACCEL"
        };

        if (!File.Exists(paramFilePathThis))
        {
            string headerLine = string.Join(",", headers);
            File.WriteAllText(paramFilePathThis, $"{headerLine}\n");

            // Display section:
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("SetupDataFile(): created PARAMETER file [" + paramFilePathThis + "]\n");
        }
    }

    private void SaveDataEntry(
        string dataFilePathThis,
        float t, 
        float dt, 
        DistalComm.ExerciseData distal_data,
        MotorbikeController.BikeCoords bike_coords_data,
        MotorbikeController.TrackCoords track_coords_data, 
        MotorbikeController.BikeInput bike_input_data, 
        MotorbikeController.BikePose bike_pose_data,
        MotorbikeController.FeedbackControl fbk_ctrl_data,
        MotorbikeController.SteerCalc steer_calc_data,
        MotorbikeController.PerformanceVars perform_vars_data)
    {
        string t_step_str = t.ToString("F3");
        string dt_step_str = dt.ToString("F3");

        /////////////////////////////////////////////////////////////////////////
        // Force sensor readings (TODO: work out this weird nomenclature):
        /////////////////////////////////////////////////////////////////////////

        float force_x_l = distal_data.ForceX;
        float force_y_l = distal_data.ForceY;

        float force_x_r = distal_data.ForceR;
        float force_y_r = distal_data.TorqueP;

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
                $"{RHBCtrlBike.instance.pos_rot_eq_ref}," +
                $"{RHBCtrlBike.instance.torque_assist}," +

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
                $"{track_coords_data.ang_ctrline_tang}," +
                $"{track_coords_data.dist_ctrline_near}," +

                $"{bike_input_data.steer_scaled}," +
                $"{bike_input_data.throttle}," +
                $"{bike_input_data.acceleration}," +

                $"{bike_pose_data.angle_roll_bike}," +
                $"{bike_pose_data.dt_angle_roll_bike}," +
                $"{bike_pose_data.angle_steer_wheel_fwd}," +

                $"{fbk_ctrl_data.pos_preview.x}," +
                $"{fbk_ctrl_data.pos_preview.z}," +
                $"{fbk_ctrl_data.pos_track_targ.x}," +
                $"{fbk_ctrl_data.pos_track_targ.z}," +
                $"{fbk_ctrl_data.angle_roll_targ}," +
                $"{fbk_ctrl_data.dt_angle_roll_targ}," +
                $"{fbk_ctrl_data.input_steer_targ}," +
                $"{fbk_ctrl_data.sin_dev_targ}," +
                $"{fbk_ctrl_data.vect_ctrline_tangent_targ.x}," +
                $"{fbk_ctrl_data.vect_ctrline_tangent_targ.z}," +
                $"{fbk_ctrl_data.err_pos_preview2targ_vect.x}," +
                $"{fbk_ctrl_data.err_pos_preview2targ_vect.z}," +
                $"{fbk_ctrl_data.err_pos_preview2targ_val}," +
                $"{fbk_ctrl_data.curv_track_targ}," +
                $"{fbk_ctrl_data.angle_roll_gain_lo}," +
                $"{fbk_ctrl_data.angle_roll_gain_hi}," +

                // $"{steer_calc_data.steer_update[0]}," +   
                // $"{steer_calc_data.steer_update[1]}," +   
                // $"{steer_calc_data.steer_update[2]}," +   
                // $"{steer_calc_data.steer_update[3]}," +   
                // $"{steer_calc_data.factor_steer_bike_speed}," +
                // $"{steer_calc_data.steer_term_input}," +
                // $"{steer_calc_data.steer_term_angle_ctrl}," +
                // $"{steer_calc_data.steer_term_dt_angle_ctrl}, " +

                $"{force_x_l}," +
                $"{force_y_l}," +
                $"{force_x_r}," +
                $"{force_y_r}," +

                $"{perform_vars_data.angle_roll_steer_equiv}," +
                $"{perform_vars_data.step_count_understeer}," +
                $"{perform_vars_data.step_count_moving}," +
                $"{RHBCtrlBike.instance.step_count}," +
                $"{MotorbikeController.instance.dist_traveled}";

            lock (fileLock)
            {
                File.AppendAllText(dataFilePathThis, $"{output}\n");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error saving data: {ex.Message}");
        }
    }

    private void SaveParametersEntry(string paramFilePathThis)
    {
        try
        {
            string output =
                $"{RHBCtrlBike.DT_STEP_APP_MSEC}," +
                $"{RHBCtrlBike.instance.USE_BEGINNER_BIKE_CONSTR}," +
                $"{RHBCtrlBike.instance.CASE_CTRL_MODE}," +
                $"{RHBCtrlBike.instance.POS_RADIAL_THROT_ZERO}," +
                $"{RHBCtrlBike.instance.K_STIFF_RADIAL_THROT_MANUAL}," +
                $"{RHBCtrlBike.instance.SPEED_AUTO_THROTTLE_MAX_KPH}," +
                $"{RHBCtrlBike.instance.FORCE_GAIN_RADIAL}," +
                $"{RHBCtrlBike.instance.FORCE_GAIN_ROT}," +
                $"{RHBCtrlBike.instance.K_STIFF_RADIAL_THROT_AUTO}," +
                $"{RHBCtrlBike.instance.K_STIFF_ROT_BASE}," +
                $"{RHBCtrlBike.instance.K_STIFF_ROT_TRACKING}," +
                $"{RHBCtrlBike.instance.B_DAMP_ROT_TRACKING}," +
                $"{RHBCtrlBike.instance.FRAC_ASSIST_STIFF}," +
                $"{RHBCtrlBike.TORQUE_ASSIST_STEER_MAX}," +
                $"{RHBCtrlBike.instance.FACT_ASSIST_STEER}," +
                $"{RHBCtrlBike.GAME_LEVEL_MID}," +
                $"{RHBCtrlBike.FACT_ASSIST_MID}," +
                $"{RHBCtrlBike.instance.FRAC_POS_ROT_INPUT_PATIENT}," +
                $"{RHBCtrlBike.instance.FACT_ASSIST_THROTTLE}," +
         
                $"{MotorbikeController.DT_PREVIEW}," +
                $"{MotorbikeController.P_GAIN_ASSIST}," +
                $"{MotorbikeController.P_GAIN_TRACK}," +
                $"{MotorbikeController.P_GAIN_LO}," +
                $"{MotorbikeController.FACT_ASSIST_STEER_MAX}," +
                $"{MotorbikeController.OFFS_FACT_ASSIST_STEER}," +
                $"{MotorbikeController.TORQUE_MOTOR_MAX}," +
                $"{MotorbikeController.instance.FACTOR_ACCEL}";


            lock (fileLock)
            {
                File.AppendAllText(paramFilePathThis, $"{output}\n");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error saving data: {ex.Message}");
        }
    }

    public void SetupRecordingEvents()
    {
        RHBCtrlBike.instance.OnExerciseStart += StartDataRecording;
        RHBCtrlBike.instance.OnExerciseStop += StopDataRecording;
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
        ////////////////////////////////////////////////////////////////
        // Launch stop data log thread:
        ////////////////////////////////////////////////////////////////
        
        StopDataRecording();

        ////////////////////////////////////////////////////////////////
        // Date/time string for files:
        ////////////////////////////////////////////////////////////////
        
        string dateTimeStr  = DateTimeStamp();

        ////////////////////////////////////////////////////////////////
        // Set up DATA file: 
        ////////////////////////////////////////////////////////////////
        
        string dataFileName = DATA_FILENAME_DEF + dateTimeStr + FILE_EXT;
        dataFilePath        = DATA_FILE_DIR + dataFileName;

        // Create new DATA csv file:
        UnityMainThreadDispatcher.Instance().Enqueue(() => SetupDataFile(dataFilePath));

        ////////////////////////////////////////////////////////////////
        // Set up PARAMETERS file:
        ////////////////////////////////////////////////////////////////
        
        string paramFileName = PARAM_FILENAME_DEF + dateTimeStr + FILE_EXT;
        paramFilePath        = DATA_FILE_DIR + paramFileName;

        // Create new PARAMETERS csv file:
        UnityMainThreadDispatcher.Instance().Enqueue(() => SetupParametersFile(paramFilePath));

        ////////////////////////////////////////////////////////////////
        // Record PARAMETERS values in file:
        ////////////////////////////////////////////////////////////////

        UnityMainThreadDispatcher.Instance().Enqueue(() => SaveParametersEntry(paramFilePath));

        ////////////////////////////////////////////////////////////////
        // 'Race started' flag:
        ////////////////////////////////////////////////////////////////

        isRaceStarted = true;

        ////////////////////////////////////////////////////////////////
        // Reset data received counter:
        ////////////////////////////////////////////////////////////////

        data_recv_count = 0;

        ////////////////////////////////////////////////////////////////
        // Set up DATA recording timer:
        ////////////////////////////////////////////////////////////////
        
        threadTimerData = new Thread(() =>
        {
            timerData = new System.Timers.Timer(DT_STEP_DATA_FBK_MSEC);
            timerData.Elapsed += SaveDataOnTimerElapsed;
            timerData.AutoReset = true;
            timerData.Start();
        });
        threadTimerData.Start();
    }

    private void StopDataRecording()
    {
        threadTimerData?.Join(); // removed unsafe Abort() call (27.10.2025)
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

                uptime_msec = RHBCtrlBike.instance.distal_data.UptimeMs;

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
                        dataFilePath,
                        t_step, 
                        dt_step,
                        RHBCtrlBike.instance.distal_data,
                        MotorbikeController.instance.bike_coords_data,
                        MotorbikeController.instance.track_coords_data,
                        MotorbikeController.instance.bike_input_data,
                        MotorbikeController.instance.bike_pose_data,
                        MotorbikeController.instance.fbk_ctrl_data,
                        MotorbikeController.instance.steer_calc_data,
                        MotorbikeController.instance.perform_vars_data);

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