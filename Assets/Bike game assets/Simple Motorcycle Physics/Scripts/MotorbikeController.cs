using Articares.Distal;
using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using System.Threading;
using System.Threading.Tasks;

public class MotorbikeController : MonoBehaviour
{
    float FACT_DEG_2_RAD = (float)Math.PI / 180f;
    const float G_ACCEL = 9.8f; // gravity acceleration in m/s

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering - Feedback control (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Preview-ahead time - CRITICAL (26.08.2025):
    public const float DT_PREVIEW = 2.0f; //  1.3f; //

    // Gain(s) for tracking target position - CRITICAL
    // Highest tested values that guarantee bike stability (25.09.2025):
    public const float P_GAIN_ASSIST   = 0.08f;
    public const float P_GAIN_TRACK    = 0.06f;

    // Gain limits for PERFORMANCE metric - steering:
    public const float P_GAIN_LO = 0.05f;
    public const float P_GAIN_HI = 0.10f;

    private float P_GAIN_ERR_POS_TARG;

    const float D_GAIN_ERR_POS_TARG = 0f;
    const float I_GAIN_ERR_POS_TARG = 0f;

    // Gain(s) for tracking reference roll angle - CRITICAL:
    const float P_GAIN_ANGLE_INPUT = 3.5f; // highest tested gain that guarantees bike stability 
    const float D_GAIN_ANGLE_INPUT = 0f;
    const float I_GAIN_ANGLE_INPUT = 0f;

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering - Forward wheel input (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Factor for steer input - formerly called STEER_INPUT_SENSITIVITY:
    const float FACTOR_STEER_INPUT = 7.0e-2f; // 3.5e-2f;  

    // Steer factor control angle:
    const float FACTOR_STEER_ANGLE_CTRL = 8.4e-2f;  

    // Steer factor for control angular speed - higher values stabilize bike:
    const float FACTOR_STEER_DT_ANGLE_CTRL = 12.0e-2f;  

    // Steer factor for control angular speed squared:
    const float FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER = 2.3f;

    // Forward wheel control:
    const float FACTOR_ANGLE_WHEEL_FWD     = 60f;

    const float RATIO_ANG_ROLL_2_ANG_WHEEL = 0.030f; // for USE_CONSTRAINED_STEER option
    const float SPEED_TRANSITION_UPRIGHT   = 1.0f; // transition speed for wheel steering angle behavior

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - STEERING - Factor adjustment:
    ////////////////////////////////////////////////////////////////////////////

    public const float FACT_ASSIST_STEER_MAX      = 0.9f;

    // Sensitivity of bike steering to RHB's ASSISTED control stiffness
    public const float OFFS_FACT_ASSIST_STEER = 0.5f; // 0.25f; 

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Throttle (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Throttle - input geometry settings:
    const float DIST_RADIAL_THROT_FULL_MM  = 2.0f; // grippers travel distance for full throttle (mm)  

    // Throttle input limits - function of RADIAL stiffness
    const float INPUT_THROT_MAX            = 1.5f; // 2.0f // NOTE: 2.0 equals about about 300 kph
    const float INPUT_THROT_UPRIGHT_THRESH = 0.6f; // minimum torque to prevent UprightForce() from kicking in

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Motor torque & acceleration:
    ////////////////////////////////////////////////////////////////////////////

    public const float TORQUE_MOTOR_MAX = 600f; // 500f; // 

    // Acceleration factor: CRITICAL value - increases top speed but can make turning harder
    public float FACTOR_ACCEL;

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Other:
    ////////////////////////////////////////////////////////////////////////////

    const float RADIUS_WHEEL = 0.7f;

    // Roll and nonslip limit angles:
    const float ANGLE_ROLL_NONSLIP_MAX_DEG = 65f; // 50f; // 

    // Refernce speeds:
    public float SPEED_REF_LOW  =  8.0f; // used by EngineSoundManager
    public float SPEED_REF_HIGH = 25.0f;

    ////////////////////////////////////////////////////////////////////////////
    // Object instance:
    ////////////////////////////////////////////////////////////////////////////

    public static MotorbikeController instance;

    ////////////////////////////////////////////////////////////////////////////
    // Bike states during race (with initial values):
    ////////////////////////////////////////////////////////////////////////////

    public float rpm_value;
    public Vector3 velocity_rel_collision;

    // Error of preview position wrt target position:
    private float err_pos_preview2targ_prev = 0f;
    private float int_err_pos_preview2targ = 0f;

    // TARGET bike roll angle / ang vel / integral for steering - CRITICAL:
    private float angle_roll_targ_prev = 0f;
    private float int_angle_roll_targ = 0f;

    // ACTUAL bike roll angle / ang vel / integral for steering - CRITICAL:
    private float angle_roll_bike = 0f;
    private float angle_roll_bike_prev = 0f;
    private float int_angle_roll_bike = 0f;

    private float dt_angle_roll_bike = 0f;

    // Additional states
    private float factor_steer_bike_speed = 0f;

    private float torque_motor = 0f;

    public bool bike_fallen      = false;
    public bool bike_fallen_prev = false;

    private bool hard_hit        = false;

    public int gear_curr = 1;

    ///////////////////////////////////////////////////////////
    // Wheel data class:
    ///////////////////////////////////////////////////////////

    public class WheelData
    {
        public WheelData(Transform transform_this, WheelCollider collider_this)
        {
            wheelTransform = transform_this;
            wheelCollider = collider_this;
            wheelStartPos = transform_this.transform.localPosition;
        }

        public Transform wheelTransform;
        public WheelCollider wheelCollider;
        public Vector3 wheelStartPos;
        public float rotation = 0f;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Wheel objects:
    ////////////////////////////////////////////////////////////////////////////

    private const int N_WHEELS = 2;

    private const int IDX_WHEEL_FWD = 0;
    private const int IDX_WHEEL_BACK = 1;

    // Wheel colliders:
    public WheelCollider wheel_coll_fwd;
    public WheelCollider wheel_coll_back;

    // Wheel transforms:
    public Transform wheelF;
    public Transform wheelB;

    // Wheel structures:
    private WheelData[] wheel_structs;

    ////////////////////////////////////////////////////////////////////////////
    // Other bike 'physical' parts:
    ////////////////////////////////////////////////////////////////////////////

    // Handles:
    [SerializeField] public GameObject handles;

    // Mudguard:
    [SerializeField] public GameObject RearMudGuard;
    public Vector3 RearMudGuardSusOffset;

    ////////////////////////////////////////////////////////////////////////////
    // Bike spatial transform and rigid body:
    ////////////////////////////////////////////////////////////////////////////

    private Transform thisTransform;
    public Vector3 com;
    private Rigidbody rigid_body;

    ///////////////////////////////////////////////////////////
    // Rider parameters:
    ///////////////////////////////////////////////////////////

    public GameObject Rider;

    public GameObject Ragdoll;
    public GameObject RagdollAnimation;

    private GameObject RagdollClone;
    private GameObject RagdollAnimationClone;

    ///////////////////////////////////////////////////////////
    // Timers and counters:
    ///////////////////////////////////////////////////////////

    private int step_count_prev = 0;

    /////////////////////////////////////////////////////////// 
    // Data display:
    /////////////////////////////////////////////////////////// 

    private int DT_DISP_DATA_MSEC = 1000;

    /////////////////////////////////////////////////////////////////////////
    // Data structures:
    /////////////////////////////////////////////////////////////////////////
    ///
    public struct BikeCoords
    {
        public Vector3 pos_bike;
        public Vector3 dt_pos_bike;
        public Vector3 dir_unit_bike;
    }

    ///////////////////////////////////////////////////////////
    // Motorbike input struct:
    ///////////////////////////////////////////////////////////

    public struct BikeInput
    {
        public float steer_scaled;
        public float throttle;
        public float acceleration;
        public float brakeForward;
        public float brakeBack;
    }

    ///////////////////////////////////////////////////////////
    // Motorbike pose (orientation angles) struct:
    ///////////////////////////////////////////////////////////
    ///
    public struct BikePose
    {
        public float angle_roll_bike;
        public float dt_angle_roll_bike;
        public float angle_steer_wheel_fwd;
    }

    /////////////////////////////////////////////////////////////
    // Track coordinates struct:
    /////////////////////////////////////////////////////////////   

    public struct TrackCoords
    {
        public Vector3 pos_ctrline_near;
        public Vector3 vect_ctrline_tang;
        public float ang_ctrline_tang;
        public float dist_ctrline_near;
    }

    /////////////////////////////////////////////////////////////
    // Steering input struct:
    /////////////////////////////////////////////////////////////   

    const int N_STEER_UPDATES = 3;

    public struct SteerCalc
    {
        public float[] steer_update;
        public float factor_steer_bike_speed;
        public float steer_term_input;
        public float steer_term_angle_ctrl;
        public float steer_term_dt_angle_ctrl;

        public SteerCalc(float[] steer_update_val)
        {
            steer_update = steer_update_val;
            factor_steer_bike_speed = 0f;
            steer_term_input = 0f;
            steer_term_angle_ctrl = 0f;
            steer_term_dt_angle_ctrl = 0f;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Auto-steer control struct:
    ////////////////////////////////////////////////////////////////////////////

    public struct FeedbackControl
    {
        public Vector3 pos_preview;
        public Vector3 pos_track_targ;
        public float angle_roll_targ;
        public float dt_angle_roll_targ;
        public float input_steer_targ;
        public float sin_dev_targ;
        public Vector3 vect_ctrline_tangent_targ;
        public Vector3 err_pos_preview2targ_vect;
        public float err_pos_preview2targ_val;
        public float curv_track_targ;
        public float angle_roll_gain_lo;
        public float angle_roll_gain_hi;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Performance-related variables struct (25.09.2025):
    ////////////////////////////////////////////////////////////////////////////

    public struct PerformanceVars 
    {
        public float angle_roll_steer_equiv;
        public int step_count_understeer;
    }

    /////////////////////////////////////////////////////////////
    // Public DATA VARIABLES for sharing data among classes:
    /////////////////////////////////////////////////////////////   

    public BikeCoords bike_coords_data = new(); // Motorbike coordinates
    public BikeInput bike_input_data = new(); // Motorbike input
    public BikePose bike_pose_data = new(); // Motorbike pose
    public TrackCoords track_coords_data = new(); // Track coordinates
    public SteerCalc steer_calc_data = new SteerCalc(new float[N_STEER_UPDATES + 1]); // steer input computations
    public FeedbackControl fbk_ctrl_data = new();
    public PerformanceVars perform_vars_data = new();

    ////////////////////////////////////////////////////////////////////////////
    // Bike coordinates:
    ////////////////////////////////////////////////////////////////////////////

    static Vector3 NULL_VECTOR3 = Vector3.zero;
    const float NULL_VALUE = 0f;

    private Vector3 pos_bike = NULL_VECTOR3;
    private Vector3 dt_pos_bike = NULL_VECTOR3;
    private Vector3 dir_unit_bike = NULL_VECTOR3;

    private float dt_pos_bike_magn = NULL_VALUE;

    /////////////////////////////////////////////////////////////
    // Previous states:
    /////////////////////////////////////////////////////////////

    private Vector3 pos_bike_prev = new();
    private float input_steer_prev = 0f;
    private float dt_pos_bike_magn_prev = 0f;

    /////////////////////////////////////////////////////////////
    // PERFORMANCE-related variables:
    /////////////////////////////////////////////////////////////

    // UNDERSTEER events counter:
    public int step_count_understeer = 0;

    // Count falls during exercise:
    public int step_count_fall = 0;

    // Distance traveled during exercise:
    public float dist_traveled = 0f;

    /////////////////////////////////////////////////////////////
    // Display settings:
    /////////////////////////////////////////////////////////////

    private bool DISP_MOTOR_CONTROL_ON = false;

    //////////////////////////////////////////////////////////////
    /// Bike speed text:
    //////////////////////////////////////////////////////////////

    [SerializeField] public Text SpeedTxt;

    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    // METHODS START HERE
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    void Awake()
    {
        instance = this;
    }

    void Start()
    {
        // Initialize wheels data:
        wheel_structs = new WheelData[N_WHEELS];

        wheel_structs[IDX_WHEEL_FWD] = new WheelData(wheelF, wheel_coll_fwd);
        wheel_structs[IDX_WHEEL_BACK] = new WheelData(wheelB, wheel_coll_back);

        // Spatial transform and rigid body:
        thisTransform = GetComponent<Transform>();
        rigid_body = GetComponent<Rigidbody>();
        rigid_body.centerOfMass = com;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Real-time update (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    void FixedUpdate()
    {
        bool reset_prev = false;
        int step_count = RHBCtrlBike.instance.step_count;

        ////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////

        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////
        // Check if bike is balanced:
        ////////////////////////////////////////////////////////////////

        uprightCheck(out bike_fallen, hard_hit);

        ////////////////////////////////////////////////////////////////
        // If bike is balanced, process control inputs and update bike's kinematic state - CRITICAL:
        ////////////////////////////////////////////////////////////////

        if (!bike_fallen)
        {
            ////////////////////////////////////////////////////////////////
            // Check if there was a reset in previous step
            // This allows zeroing the throttle and fwd force after a collision(30.10.2025):
            ////////////////////////////////////////////////////////////////
            
            if (bike_fallen_prev)
                reset_prev = true;
            else
                reset_prev = false;

            ////////////////////////////////////////////////////////////////
            // Generate bike motion and control states:
            ////////////////////////////////////////////////////////////////
            
            BikeMotionAndControlStates(step_count, RHBCtrlBike.instance.distal_data,
                ref bike_coords_data,
                ref bike_input_data,
                ref bike_pose_data,
                ref track_coords_data,
                ref steer_calc_data,
                ref fbk_ctrl_data,
                reset_prev);

            ////////////////////////////////////////////////////////////////
            // Update wheels rotation - CRITICAL: 
            ////////////////////////////////////////////////////////////////

            updateWheels(ref wheel_structs[IDX_WHEEL_FWD], dt_step);
            updateWheels(ref wheel_structs[IDX_WHEEL_BACK], dt_step);

            ////////////////////////////////////////////////////////////////
            // Additional bike dynamics updates:
            ////////////////////////////////////////////////////////////////

            RearMudGuardSuspension();

            bool USE_MOTOR_DISENGAGE = false; // TODO: test if this makes acceleration smoother
            CalcGear(ref gear_curr, ref rpm_value, dt_pos_bike_magn, USE_MOTOR_DISENGAGE);

            ////////////////////////////////////////////////////////////////
            // PERFORMANCE variables 1: UNDERSTEER fraction
            ////////////////////////////////////////////////////////////////

            // Steering input: EQUIVALENT roll angle as function of steering input (TODO: test approach)
            float angle_roll_steer_equiv = bike_input_data.steer_scaled;

            // UNDERSTEER detection: compare EQUIVALENT roll angle to
            // minimum (low-gain) steering input required to keep bike on track
            if (Math.Abs(angle_roll_steer_equiv) < Mathf.Abs(fbk_ctrl_data.angle_roll_gain_lo))
                step_count_understeer++;

            ////////////////////////////////////////////////////////////////
            // PERFORMANCE variables 2: DISTANCE TRAVELED so far
            ////////////////////////////////////////////////////////////////

            float dist_traveled_rel; // distance relative to start line (wraps around)

            if (RHBCtrlBike.instance.RACE_DIRECTION == RHBCtrlBike.DIR_CW)
                dist_traveled_rel = Track.instance.GetDistanceAtPosition(bike_coords_data.pos_bike);
            else
                dist_traveled_rel = Track.instance.GetTrackLength() - Track.instance.GetDistanceAtPosition(bike_coords_data.pos_bike);

            dist_traveled = Mathf.Max(dist_traveled, dist_traveled_rel);

            ////////////////////////////////////////////////////////////////
            // Update PERFORMANCE public DATA VARIABLES for sharing among other classes (ensures atomicity & real-time updating):
            ////////////////////////////////////////////////////////////////   

            perform_vars_data.angle_roll_steer_equiv = angle_roll_steer_equiv;
            perform_vars_data.step_count_understeer = step_count_understeer;
        }
        else if (!bike_fallen_prev)
            step_count_fall++;

        // Store bike fallen state for next iteration:
        bike_fallen_prev = bike_fallen;

        ////////////////////////////////////////////////////////////////
        // Reset after fall:
        ////////////////////////////////////////////////////////////////

        if (Input.GetKey(KeyCode.R) || bike_fallen)
            Reset(ref bike_fallen, ref hard_hit);

        ////////////////////////////////////////////////////////////////
        // Store current step count for next iteration:
        ////////////////////////////////////////////////////////////////

        step_count_prev = step_count;

        ////////////////////////////////////////////////////////////////
        // Update text for speed display in Unity (21.08.2025):
        ////////////////////////////////////////////////////////////////       

        // rigid_body.velocity magnitude shows speed meters per second (m/s):     
        SpeedTxt.text = ConvertSpeedMStoKMH(MagnitudeXZ(rigid_body.velocity)).ToString("F0");

        ////////////////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////////////////

        if (step_count % (DT_DISP_DATA_MSEC / RHBCtrlBike.DT_STEP_APP_MSEC) == 0
            && DISP_MOTOR_CONTROL_ON)
        {
            // Time elapsed display:
            float timeElapsedValue   = RHBCtrlBike.instance.timeElapsedValue;
            TimeSpan timeElapsedSpan = TimeSpan.FromSeconds(timeElapsedValue);

            string timeElapsedText   = String.Format("{0:#00}", timeElapsedSpan.Minutes) + ":" + String.Format("{0:#00}", timeElapsedSpan.Seconds);

            ExternalConsoleLogger.Log("Update(" + step_count + ") t [" + String.Format("{0:#0.000}", timeElapsedValue) + "]:");
            ExternalConsoleLogger.Log("   pos_bike       " + bike_coords_data.pos_bike);
            ExternalConsoleLogger.Log("   pos_preview    " + fbk_ctrl_data.pos_preview);
            ExternalConsoleLogger.Log("   pos_track_targ " + fbk_ctrl_data.pos_track_targ);
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("   angle_roll_bike        [" + String.Format("{0:#0.000}", angle_roll_bike) + "]");
            ExternalConsoleLogger.Log("   angle_roll (targ,bike) [" + String.Format("{0:#0.000}", fbk_ctrl_data.angle_roll_targ) + "] [" + String.Format("{0:#0.000}", angle_roll_bike) + "]");
            ExternalConsoleLogger.Log("   input_steer_targ       [" + String.Format("{0:#0.000}", fbk_ctrl_data.input_steer_targ) + "]");
            ExternalConsoleLogger.Log(" ");
        }
    }

    ////////////////////////////////////////////////////////////////
    // Bike state: control inputs and kinematics - CRITICAL:
    ////////////////////////////////////////////////////////////////
    
    void BikeMotionAndControlStates(int step_count, DistalComm.ExerciseData distal_this,
        ref BikeCoords bike_coords_this,
        ref BikeInput bike_input_this,
        ref BikePose bike_pose_this,
        ref TrackCoords track_coords_this,
        ref SteerCalc steer_calc_this,
        ref FeedbackControl fbk_ctrl_this,
        bool reset_prev)
    {
        ////////////////////////////////////////////////////////////////
        // Bike - local variables:
        ////////////////////////////////////////////////////////////////

        BikeInput bike_input = new();

        ////////////////////////////////////////////////////////////////
        // RHB inputs:
        ////////////////////////////////////////////////////////////////

        float pos_radial = distal_this.PositionR; // RADIAL position
        float pos_rot    = distal_this.PositionP; // ROTATIONAL position

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // BIKE INPUT 1: ACCELERATION
        ////////////////////////////////////////////////////////////////   
        ////////////////////////////////////////////////////////////////      

        bike_input.acceleration = 0f;

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // BIKE INPUT 2: THROTTLE
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////

        if (RHBCtrlBike.instance.isExerciseStarted && !reset_prev)
        {

            bike_input.throttle = InputThrottleCases(pos_radial, MotorbikeController.instance, RHBCtrlBike.instance.CASE_CTRL_MODE);

            if (MagnitudeXZ(bike_coords_this.dt_pos_bike) > SPEED_TRANSITION_UPRIGHT // was bike_input.throttle > 0
                && RHBCtrlBike.instance.UPRIGHT_CONSTR_ON == true)
            {
                uprightConstraintRemove(out RHBCtrlBike.instance.UPRIGHT_CONSTR_ON); // constraint flag (13.09.2025)

                // Display section:
                if (DISP_MOTOR_CONTROL_ON)
                {
                    ExternalConsoleLogger.Log("_________________________________________________________________");
                    ExternalConsoleLogger.Log("BikeMotionAndControlStates(): upright constraint [FALSE] \n");
                }
            }
        }
        else
        {
            ////////////////////////////////////////////////////////////////
            // NOTE: (reset_prev == true) allows zeroing the throttle and fwd force after a collision(30.10.2025):
            ////////////////////////////////////////////////////////////////
            
            bike_input.throttle = 0f;
            rigid_body.AddForce(Vector3.zero);

        }

        ////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////
        // BIKE INPUT 3: STEERING
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////   

        ////////////////////////////////////////////////////////////////////////////
        // Steering input case 1: FEEDBACK-BASED (target)
        //
        // Updates public DATA VARIABLES for sharing among other classes (for atomicity & real-time updating)
        // NOTE: input_steer_targ is also contained in fbk_ctrl_this
        ////////////////////////////////////////////////////////////////////////////

        float input_steer_targ;

        if (RHBCtrlBike.instance.isExerciseStarted && MotorbikeController.instance != null && Track.instance != null)
            input_steer_targ = InputSteerTargetFeedback(ref bike_coords_this, ref fbk_ctrl_this);
        else
            input_steer_targ = 0f;

        ////////////////////////////////////////////////////////////////
        // Steering input case 2: MANUAL (from RHB ROTATIONAL input)
        ////////////////////////////////////////////////////////////////      

        float input_steer_manual = 1.0f / RHBCtrlBike.instance.FRAC_POS_ROT_INPUT_PATIENT * pos_rot;

        ////////////////////////////////////////////////////////////////
        // Select steering mode:
        ////////////////////////////////////////////////////////////////

        if (RHBCtrlBike.instance.isExerciseStarted)
            bike_input.steer_scaled = InputSteerCases(
                input_steer_manual, input_steer_targ, RHBCtrlBike.instance.CASE_CTRL_MODE);
        else
            bike_input.steer_scaled = 0f;

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // BIKE CONTROL ACTIONS: several steps
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        // Bike control commands - CRITICAL:
        ////////////////////////////////////////////////////////////////           

        MotorbikeControl(bike_input, step_count, out bike_coords_this, out bike_pose_this, ref steer_calc_this);

        ////////////////////////////////////////////////////////////////
        // Adjust torque (key input mode only) and wheel sideways friction:
        ////////////////////////////////////////////////////////////////

        SetWheelFrictionVelocityBased();

        ////////////////////////////////////////////////////////////////
        // Update handles relative angle in transform:
        ////////////////////////////////////////////////////////////////

        steerHandles();

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // Update other public DATA VARIABLES for sharing among other classes (ensures atomicity & real-time updating):
        ////////////////////////////////////////////////////////////////          
        ////////////////////////////////////////////////////////////////

        // Bike input:
        bike_input_this = bike_input;

        // Track coordinates:
        track_coords_this = GetTrackCoordsRelToBike(bike_coords_this.pos_bike);
    }

    ////////////////////////////////////////////////////////////////
    // Bike Input - Throttle commands - CASES:
    ////////////////////////////////////////////////////////////////  

    float InputThrottleCases(float pos_radial, MotorbikeController bike_controller, int case_ctrl_mode)
    {
        float input_throttle;

        ////////////////////////////////////////////////////////////////
        // Deviation from centerline target (several uses):  
        ////////////////////////////////////////////////////////////////    

        float sin_dev_targ = bike_controller.fbk_ctrl_data.sin_dev_targ;

        ////////////////////////////////////////////////////////////////
        // AUTO THROTTLE input - feedback based:
        ////////////////////////////////////////////////////////////////

        float factor_speed_throttle = 1f - (float)Math.Abs(sin_dev_targ); // was: 1f - sin_dev_targ*sin_dev_targ;

        float input_throttle_fbk = factor_speed_throttle * RHBCtrlBike.instance.SPEED_AUTO_THROTTLE_MAX_KPH / 100f;

        ////////////////////////////////////////////////////////////////
        // MANUAL THROTTLEe input - from RHB:
        ////////////////////////////////////////////////////////////////

        float pos_throttle      = pos_radial;
        float pos_throttle_zero = RHBCtrlBike.instance.POS_RADIAL_THROT_ZERO;

        float SCALE_INPUT_THROTTLE = 1000f / DIST_RADIAL_THROT_FULL_MM;

        float input_throttle_manual = Mathf.Clamp(
            SCALE_INPUT_THROTTLE * (pos_throttle_zero - pos_throttle), 
            0f, INPUT_THROT_MAX);

        ////////////////////////////////////////////////////////////////
        // Select throttle input:
        ////////////////////////////////////////////////////////////////

        switch (case_ctrl_mode)
        {
            case RHBCtrlBike.CTRL_ASSISTED:

                input_throttle =
                            RHBCtrlBike.instance.FACT_ASSIST_THROTTLE  * input_throttle_fbk
                    + (1f - RHBCtrlBike.instance.FACT_ASSIST_THROTTLE) * input_throttle_manual;
                break;

            case RHBCtrlBike.CTRL_AUTO_STEER_AUTO_THROT:

                input_throttle = input_throttle_fbk;
                break;

            case RHBCtrlBike.CTRL_AUTO_STEER_MANUAL_THROT:
            case RHBCtrlBike.CTRL_MANUAL_SIMPLE:

                input_throttle = input_throttle_manual;
                break;

            default:

                input_throttle = 0f;
                break;
        }

        return input_throttle;
    }

    ////////////////////////////////////////////////////////////////
    // Bike Input - Steering commands - CASES:
    ////////////////////////////////////////////////////////////////  

    float InputSteerCases(float input_steer_manual, float input_steer_targ, int case_ctrl_mode)
    {
        ////////////////////////////////////////////////////////////////
        // Reference steering input (several cases):
        ////////////////////////////////////////////////////////////////

        float input_steer_ref;

        switch (case_ctrl_mode)
        {
            case RHBCtrlBike.CTRL_ASSISTED:

                float ratio_fact_assist =
                    (FACT_ASSIST_STEER_MAX - OFFS_FACT_ASSIST_STEER) / FACT_ASSIST_STEER_MAX;

                float FACT_ASSIST_STEER_ADJ = ratio_fact_assist * RHBCtrlBike.instance.FACT_ASSIST_STEER;

                // Compute steering input:
                input_steer_ref =
                           FACT_ASSIST_STEER_ADJ  * input_steer_targ
                   + (1f - FACT_ASSIST_STEER_ADJ) * input_steer_manual;
                break;

            case RHBCtrlBike.CTRL_AUTO_STEER_AUTO_THROT:
            case RHBCtrlBike.CTRL_AUTO_STEER_MANUAL_THROT:

                input_steer_ref = input_steer_targ;
                break;

            case RHBCtrlBike.CTRL_MANUAL_SIMPLE:

                input_steer_ref = input_steer_manual;
                break;

            default:

                input_steer_ref = 0f;
                break;
        }

        return input_steer_ref;
    }

    ////////////////////////////////////////////////////////////////
    // Bike steering - Auxiliary functions:
    ////////////////////////////////////////////////////////////////

    float InputSteerTargetFeedback(ref BikeCoords bike_coords, ref FeedbackControl fbk_ctrl)
    {
        // Bike coordinates: 
        Vector3 pos_bike = bike_coords.pos_bike;
        Vector3 dt_pos_bike = bike_coords.dt_pos_bike;
        Vector3 dir_unit_bike = bike_coords.dir_unit_bike;

        // Steering input (also included in fbk_ctrl):
        float input_steer_targ = NULL_VALUE;

        Vector3 pos_preview      = NULL_VECTOR3;
        Vector3 pos_track_targ   = NULL_VECTOR3;
        float angle_roll_targ    = NULL_VALUE;
        float dt_angle_roll_targ = NULL_VALUE;
        float sin_dev_targ       = NULL_VALUE; // angular deviation of bike's heading wrt to target
        Vector3 vect_ctrline_tangent_targ = NULL_VECTOR3;
        float curv_track_targ    = NULL_VALUE;
        float angle_roll_gain_lo = NULL_VALUE;
        float angle_roll_gain_hi = NULL_VALUE;

        ////////////////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////////////////

        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 1: track coordinates:
        ////////////////////////////////////////////////////////////////////////////

        TrackDataFeedbackControl(
            pos_bike, dt_pos_bike, dir_unit_bike,
            DT_PREVIEW, Track.instance,
            out pos_preview,
            out pos_track_targ,
            out vect_ctrline_tangent_targ,
            out curv_track_targ
        );

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 2: lateral displacement control - roll angle /ang vel target:
        ////////////////////////////////////////////////////////////////////////////

        // Angular deviation of bike's heading wrt to target tangent:
        Vector3 vect_unit_dev_tangent = Vector3.Cross(dir_unit_bike, vect_ctrline_tangent_targ.normalized);
        sin_dev_targ = vect_unit_dev_tangent.y;

        // Deviation relative to target point on track:
        Vector3 err_pos_preview2targ_vect = pos_track_targ - pos_preview;

        Vector3 vect_unit_turn_targ = Vector3.Cross(dir_unit_bike, err_pos_preview2targ_vect.normalized); // test vector to establish turn direction
        int sgn_turn_targ = Math.Sign(-vect_unit_turn_targ.y);

        // Error of preview position wrt target position:
        float err_pos_preview2targ = sgn_turn_targ * MagnitudeXZ(err_pos_preview2targ_vect);

        // Error of preview position: time derivative:
        float dt_err_pos_preview2targ = (err_pos_preview2targ - err_pos_preview2targ_prev) / dt_step;

        // Error of preview position: integral
        int_err_pos_preview2targ += err_pos_preview2targ * dt_step;

        // Store error for next iteration:
        err_pos_preview2targ_prev = err_pos_preview2targ;

        // Peoportional gain for target roll angle - CRITICAL
        // Highest tested values that guarantee bike stability (25.09.2025):
        if (RHBCtrlBike.instance.CASE_CTRL_MODE == RHBCtrlBike.CTRL_ASSISTED)
            P_GAIN_ERR_POS_TARG = P_GAIN_ASSIST;
        else
            P_GAIN_ERR_POS_TARG = P_GAIN_TRACK; 
  
        // TARGET roll angle computation - CRITICAL:
        angle_roll_targ =
            P_GAIN_ERR_POS_TARG   * err_pos_preview2targ
            + D_GAIN_ERR_POS_TARG * dt_err_pos_preview2targ
            + I_GAIN_ERR_POS_TARG * int_err_pos_preview2targ;

        // TARGET roll angular velocity:
        dt_angle_roll_targ = (angle_roll_targ - angle_roll_targ_prev) / dt_step;

        // TARGET roll angle integral:
        int_angle_roll_targ += angle_roll_targ * dt_step;

        // Store TARGET roll angle for next iteration:
        angle_roll_targ_prev = angle_roll_targ;

        // IDEAL roll angle for PERFORMANCE metrics (25.09.2025): 
        angle_roll_gain_lo = P_GAIN_LO * err_pos_preview2targ;
        angle_roll_gain_hi = P_GAIN_HI * err_pos_preview2targ;

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 3: steering input - KEY STEP
        ////////////////////////////////////////////////////////////////////////////

        input_steer_targ =
            P_GAIN_ANGLE_INPUT   * (angle_roll_targ - angle_roll_bike)
            + D_GAIN_ANGLE_INPUT * (dt_angle_roll_targ - dt_angle_roll_bike)
            + I_GAIN_ANGLE_INPUT * (int_angle_roll_targ - int_angle_roll_bike);

        ////////////////////////////////////////////////////////////////
        // Update data variables' struct for sharing among other classes (for atomicity & real-time updating):
        ////////////////////////////////////////////////////////////////    

        fbk_ctrl.pos_preview        = pos_preview;
        fbk_ctrl.pos_track_targ     = pos_track_targ;
        fbk_ctrl.angle_roll_targ    = angle_roll_targ;
        fbk_ctrl.dt_angle_roll_targ = dt_angle_roll_targ;
        fbk_ctrl.input_steer_targ   = input_steer_targ;
        fbk_ctrl.sin_dev_targ       = sin_dev_targ;
        fbk_ctrl.vect_ctrline_tangent_targ = vect_ctrline_tangent_targ;
        fbk_ctrl.err_pos_preview2targ_vect = err_pos_preview2targ_vect;
        fbk_ctrl.err_pos_preview2targ_val  = err_pos_preview2targ;
        fbk_ctrl.curv_track_targ           = curv_track_targ;
        fbk_ctrl.angle_roll_gain_lo        = angle_roll_gain_lo;
        fbk_ctrl.angle_roll_gain_hi        = angle_roll_gain_hi;

        return input_steer_targ;
    }

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    // Motorbike control with dynamics - CRITICAL:
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    private void MotorbikeControl(BikeInput bike_input, int step_count,
        out BikeCoords bike_coords, out BikePose bike_pose, ref SteerCalc steer_calc)
    {
        ////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////

        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////
        // Bike position and velocity:
        ////////////////////////////////////////////////////////////////

        Vector3 pos_bike      = thisTransform.position;
        Vector3 dt_pos_bike   = (pos_bike - pos_bike_prev) / dt_step;
        Vector3 dir_unit_bike = GetBikeDirectionVector();

        pos_bike_prev = pos_bike;
        dt_pos_bike_magn = MagnitudeXZ(dt_pos_bike);

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // STEERING INPUT:
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////
        // Bike roll angle (rad) - super CRITICAL:
        ////////////////////////////////////////////////////////////////

        // If Beginner bike is selected, enforce z-axis rotation to target roll angle value:
        if (RHBCtrlBike.instance.USE_BEGINNER_BIKE_CONSTR)
            thisTransform.rotation = Quaternion.Euler(
                thisTransform.rotation.eulerAngles.x,
                thisTransform.rotation.eulerAngles.y,
                1f / FACT_DEG_2_RAD * fbk_ctrl_data.angle_roll_targ);

        // Assign z rotation value to bike roll angle:
        if (thisTransform.eulerAngles.z > 180f)
            angle_roll_bike = FACT_DEG_2_RAD * thisTransform.eulerAngles.z - 2f * (float)Math.PI;
        else
            angle_roll_bike = FACT_DEG_2_RAD * thisTransform.eulerAngles.z;

        ////////////////////////////////////////////////////////////////
        // Angular roll velocity:
        ////////////////////////////////////////////////////////////////

        dt_angle_roll_bike = (angle_roll_bike - angle_roll_bike_prev) / dt_step;

        ////////////////////////////////////////////////////////////////
        // Roll angle integral:
        ////////////////////////////////////////////////////////////////

        int_angle_roll_bike += angle_roll_bike * dt_step;

        ////////////////////////////////////////////////////////////////
        // Store roll angle for next iteration:
        ////////////////////////////////////////////////////////////////

        angle_roll_bike_prev = angle_roll_bike;

        ////////////////////////////////////////////////////////////////
        // Update steering input with bike dynamics - CRITICAL
        // Also returns steer calculation data:
        ////////////////////////////////////////////////////////////////

        InputSteerUpdateDynamicsBased(ref bike_input, ref steer_calc);

        ////////////////////////////////////////////////////////////////
        // Update wheels steering angle (wheel colliders) - CRITICAL:
        ////////////////////////////////////////////////////////////////
        
        Vector3 dt_pos_bike_xz = Vector_XZ(dt_pos_bike);

        if (dt_pos_bike_magn >= SPEED_TRANSITION_UPRIGHT)
        {
            // Ig Beginner bike is selected, override dynamics-based steering:
            if (RHBCtrlBike.instance.USE_BEGINNER_BIKE_CONSTR)
                wheel_coll_fwd.steerAngle = -1f / FACT_DEG_2_RAD * RATIO_ANG_ROLL_2_ANG_WHEEL * angle_roll_bike;
            else
                wheel_coll_fwd.steerAngle = FACTOR_ANGLE_WHEEL_FWD * bike_input.steer_scaled;

            wheel_coll_fwd.steerAngle = FACTOR_ANGLE_WHEEL_FWD * bike_input.steer_scaled;
        }
        else
        {
            if (dt_pos_bike_magn < dt_pos_bike_magn_prev && RHBCtrlBike.instance.UPRIGHT_CONSTR_ON == false)
            {
                wheel_coll_fwd.steerAngle = 0f; // Mathf.Clamp(bike_input.steer_scaled, -dt_pos_bike_magn, dt_pos_bike_magn);

                try
                {
                    uprightConstraintEnforce(ref RHBCtrlBike.instance.UPRIGHT_CONSTR_ON); // constraint flag (13.09.2025)
                }
                catch
                { 
                    ExternalConsoleLogger.Log("_________________________________________________________________");
                    ExternalConsoleLogger.Log("MotorbikeController / MotorbikeControl(): CAN'T APPLY UPRIGHT CONSTRAINT \n");
                }

                // Display section:
                if (DISP_MOTOR_CONTROL_ON)
                {
                    ExternalConsoleLogger.Log("_________________________________________________________________");
                    ExternalConsoleLogger.Log("MotorbikeControl(): upright constraint [TRUE] \n");
                }
            }
        }

        ////////////////////////////////////////////////////////////////
        // Save bike velocity magnitude for next step:
        ////////////////////////////////////////////////////////////////

        dt_pos_bike_magn_prev = dt_pos_bike_magn;

        ////////////////////////////////////////////////////////////////
        // Save steering value for next step:
        ////////////////////////////////////////////////////////////////

        input_steer_prev = bike_input.steer_scaled;

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // THROTTLE INPUT:
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        
        ////////////////////////////////////////////////////////////////
        // Throttle input 1: apply input torque to back wheel - CRITICAL:
        ////////////////////////////////////////////////////////////////

        wheel_coll_back.motorTorque = torque_motor * bike_input.throttle;

        ////////////////////////////////////////////////////////////////
        // Throttle input 2: apply forward force to bike body - CRITICAL:
        ////////////////////////////////////////////////////////////////

        // For cases involving throttle feedback control, limit acceleration capability at high speeds:
        if (RHBCtrlBike.instance.CASE_CTRL_MODE == RHBCtrlBike.CTRL_ASSISTED
            && RHBCtrlBike.instance.FACT_ASSIST_THROTTLE > 0
            && dt_pos_bike_magn > SPEED_REF_HIGH)
                FACTOR_ACCEL = 1000f;

        else if (RHBCtrlBike.instance.CASE_CTRL_MODE == RHBCtrlBike.CTRL_AUTO_STEER_AUTO_THROT
            && dt_pos_bike_magn > SPEED_REF_HIGH)
                FACTOR_ACCEL = 1000f;

        else
            FACTOR_ACCEL = 2000f;

        rigid_body.AddForce(FACTOR_ACCEL * bike_input.throttle * thisTransform.forward);

        ////////////////////////////////////////////////////////////////
        // Update rigid-body Cartesian velocities:
        ////////////////////////////////////////////////////////////////

        if (Input.GetAxis("Vertical") < 0)
            rigid_body.velocity = new Vector3(
                rigid_body.velocity.x,
                rigid_body.velocity.y,
                rigid_body.velocity.z);

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        // OUTPUT STRUCTURES:
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        
        ////////////////////////////////////////////////////////////////
        // Generate bike coordinates output struct:
        ////////////////////////////////////////////////////////////////

        bike_coords.pos_bike = pos_bike;
        bike_coords.dt_pos_bike = dt_pos_bike;
        bike_coords.dir_unit_bike = dir_unit_bike;

        ////////////////////////////////////////////////////////////////
        // Generate bike pose output struct:
        ////////////////////////////////////////////////////////////////

        bike_pose.angle_roll_bike = angle_roll_bike;
        bike_pose.dt_angle_roll_bike = dt_angle_roll_bike;
        bike_pose.angle_steer_wheel_fwd = FACT_DEG_2_RAD * wheel_coll_fwd.steerAngle;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Data for feedback (assisted / auto-steer) control:
    ////////////////////////////////////////////////////////////////////////////

    private void TrackDataFeedbackControl(
        Vector3 pos_bike, Vector3 dt_pos_bike, Vector3 dir_unit_bike,
        float dt_preview, Track track_this,
        out Vector3 pos_preview_this,
        out Vector3 pos_track_targ_this,
        out Vector3 vect_ctrline_tang_target_this,
        out float curv_track_targ_this
    )
    {
        // Distance offset for curvature calculations:
        const float OFFS_DIST_CURV = 250f;

        // Project bike vectors onto the track (xz) plane - TODO: this may be redundant with uses of Magnitude_XZ() elsewhere
        const bool ENFORCE_XZ_PROJECTION = true;

        if (ENFORCE_XZ_PROJECTION)
        {
            pos_bike = Vector_XZ(pos_bike);
            dt_pos_bike = Vector_XZ(dt_pos_bike);
            dir_unit_bike = Vector_XZ(dir_unit_bike);
        }

        // Obtain preview point:
        float dist_preview = MagnitudeXZ(dt_pos_bike) * dt_preview; // distance to preview point ahead

        pos_preview_this = pos_bike + dist_preview * dir_unit_bike;

        // Obtain target point on track centerline:
        pos_track_targ_this = track_this.GetClosestPointOnCenterLine(pos_preview_this);

        // Tangent vector at target point:
        vect_ctrline_tang_target_this = track_this.GetTangentAtPosition(pos_track_targ_this);

        // Obtain centerline point closest to current bike position:
        Vector3 pos_track_bike = track_this.GetClosestPointOnCenterLine(pos_bike);

        // Get curvature at target point:        
        curv_track_targ_this = track_this.GetCurvatureAtPositionByDistance(
            pos_track_targ_this, OFFS_DIST_CURV);
    }

    ////////////////////////////////////////////////////////////////
    // Update steering input with bike dynamics:
    ////////////////////////////////////////////////////////////////

    private void InputSteerUpdateDynamicsBased(ref BikeInput bike_input, ref SteerCalc steer_calc)
    {
        // This array allows tracking the multiple updates to bike_input.steer that happen in the function:
        float[] steer_update = new float[N_STEER_UPDATES + 1];

        steer_update[0] = bike_input.steer_scaled;

        float ratio_speed;

        // Low-speed case:
        if (dt_pos_bike_magn < SPEED_REF_LOW)
        {
            ratio_speed = dt_pos_bike_magn / SPEED_REF_LOW;

            angle_roll_bike *= (2.0f - ratio_speed);
            dt_angle_roll_bike *= ratio_speed * ratio_speed;

            bike_input.acceleration += 3.0f * Mathf.Abs(angle_roll_bike) * (1.0f - ratio_speed);
            bike_input.steer_scaled *= ratio_speed * ratio_speed;
        }

        // High-speed case:
        else if (dt_pos_bike_magn > SPEED_REF_HIGH)
        {
            ratio_speed = dt_pos_bike_magn / SPEED_REF_HIGH;

            // Adjust roll angular speed for return to upright:
            if (dt_angle_roll_bike * angle_roll_bike < 0f)
                dt_angle_roll_bike *= ratio_speed; // was FACTOR_DT_ANGLE_CTRL_RETURN *
        }

        ////////////////////////////////////////////////////////////////
        // bike_input.steer UPDATE 1 (update steer input with angle_ctrl squared):   
        ////////////////////////////////////////////////////////////////         

        steer_update[1] = bike_input.steer_scaled * (1 - FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER * (float)Math.Pow(angle_roll_bike, 2));

        bike_input.steer_scaled = steer_update[1];

        ////////////////////////////////////////////////////////////////
        // bike_input.steer UPDATE 2 (steer terms weighted sum: input, angle_ctrl, dt_angle_ctrl):
        ////////////////////////////////////////////////////////////////

        float steer_term_input = FACTOR_STEER_INPUT * bike_input.steer_scaled;
        float steer_term_angle_ctrl = FACTOR_STEER_ANGLE_CTRL * angle_roll_bike;
        float steer_term_dt_angle_ctrl = FACTOR_STEER_DT_ANGLE_CTRL * dt_angle_roll_bike;

        steer_update[2] = steer_term_input - steer_term_angle_ctrl - steer_term_dt_angle_ctrl; // was factor_steer_bike_speed *

        bike_input.steer_scaled = steer_update[2];

        ////////////////////////////////////////////////////////////////
        // bike_input.steer UPDATE 3 (clamp with input_steer_prev):
        ////////////////////////////////////////////////////////////////

        // TODO: keep or discard
        /*
        const float FACTOR_INC_STEER = 20.0f; // 10.0f; //  

        float inc_steer = FACTOR_INC_STEER * dt_step;
        steer_update[3] = Mathf.Clamp(bike_input.steer, input_steer_prev - inc_steer, input_steer_prev + inc_steer);
        */

        steer_update[3] = steer_update[2];

        bike_input.steer_scaled = steer_update[3];

        ////////////////////////////////////////////////////////////////
        // Steer calculation data:  
        ////////////////////////////////////////////////////////////////

        for (int i = 0; i <= N_STEER_UPDATES; i++)
            steer_calc.steer_update[i] = steer_update[i];

        steer_calc.factor_steer_bike_speed = factor_steer_bike_speed;
        steer_calc.steer_term_input = steer_term_input;
        steer_calc.steer_term_angle_ctrl = steer_term_angle_ctrl;
        steer_calc.steer_term_dt_angle_ctrl = steer_term_dt_angle_ctrl;
    }

    ////////////////////////////////////////////////////////////////
    // Ancillary functions:
    ////////////////////////////////////////////////////////////////

    public void Reset(ref bool bike_fallen_this, ref bool hard_hit_this)
    {
        //////////////////////////////////////////////////////////////////////////////////////
        // Reset bike position to the closest center point to the bike's current position:
        //////////////////////////////////////////////////////////////////////////////////////

        Transform transf = GetComponent<Transform>();

        // transf.position = Track.instance.GetClosestPointOnCenterLine(transf.position) + new Vector3(0f, 0.1f, 0f);
        transf.position = Track.instance.GetClosestPointOnCenterLine(bike_coords_data.pos_bike) + new Vector3(0f, 0.1f, 0f);

        //////////////////////////////////////////////////////////////////////////////////////
        // Reset bike rotation to align with the rotation of the track:
        //////////////////////////////////////////////////////////////////////////////////////

        Quaternion ang_track_tang = Track.instance.GetTrackRotationAtPosition(transf.position);
        float ang_bike_yaw = ang_track_tang.eulerAngles.y;
        transf.transform.rotation = Quaternion.Euler(0f, ang_bike_yaw, 0f);

        //////////////////////////////////////////////////////////////////////////////////////
        // Reset bike states:
        //////////////////////////////////////////////////////////////////////////////////////

        rigid_body.velocity = Vector3.zero;
        rigid_body.centerOfMass = com;

        bike_fallen_this = false;
        hard_hit_this    = false;

        try
        {
            uprightConstraintEnforce(ref RHBCtrlBike.instance.UPRIGHT_CONSTR_ON); // constraint flag (13.09.2025)
        }
        catch
        {
            ExternalConsoleLogger.Log("_________________________________________________________________");
            ExternalConsoleLogger.Log("MotorbikeController / Reset(): CAN'T APPLY UPRIGHT CONSTRAINT \n");
        }

        //////////////////////////////////////////////////////////////////////////////////////
        // Reset rider states:
        //////////////////////////////////////////////////////////////////////////////////////

        Rider.SetActive(true);

        Destroy(RagdollClone);
        Destroy(RagdollAnimationClone);

        // Display section:
        if (DISP_MOTOR_CONTROL_ON) {
            ExternalConsoleLogger.Log("_________________________________________________________________");
            ExternalConsoleLogger.Log("Reset(): upright constraint [TRUE] \n");
        }
    }

    public void uprightConstraintEnforce(ref bool upright_constr_on_this)
    {
        // Enforce roll angle zero:
        thisTransform.rotation = Quaternion.Euler(
            thisTransform.rotation.eulerAngles.x,
            thisTransform.rotation.eulerAngles.y,
            0);

        angle_roll_bike = 0f; // CRITICAL

        // Rigid body constraints:
        rigid_body.angularDrag = 100f; // TODO: does this help anything?
        rigid_body.constraints = RigidbodyConstraints.FreezeRotationZ; // was FreezeAll;

        upright_constr_on_this = true;
    }

    public void uprightConstraintRemove(out bool upright_constr_on_this) {

        rigid_body.angularDrag = 0f; // TODO: does this help anything?

        rigid_body.constraints = RigidbodyConstraints.None;
        upright_constr_on_this = false; // constraint flag (13.09.2025)
    }

    void OnCollisionEnter(Collision collision)
    {
        const bool TEST_VELOCITY_REL = false;
        const float VELOCITY_REL_HARD_HIT = 3.0f;

        velocity_rel_collision = collision.relativeVelocity;
        
        if (collision.relativeVelocity.magnitude > VELOCITY_REL_HARD_HIT || !TEST_VELOCITY_REL)
            hard_hit = true;

        if (hard_hit)
        {
            ExternalConsoleLogger.Log("_________________________________________________________________");
            ExternalConsoleLogger.Log("OnCollisionEnter(): hard hit \n");
        }
    }

    public void uprightCheck(out bool bike_fallen_this, bool hard_hit_this)
    {
        if ((Mathf.Abs(angle_roll_bike) > FACT_DEG_2_RAD * ANGLE_ROLL_NONSLIP_MAX_DEG || Input.GetKeyDown(KeyCode.F) || hard_hit_this)
            ) // && bike_fallen_this == false
        {
            Rider.SetActive(false);

            RagdollClone = Instantiate(Ragdoll);
            RagdollAnimationClone = Instantiate(RagdollAnimation);

            bike_fallen_this = true;
        }
        else
            bike_fallen_this = false;
    }

    private void updateWheels(ref WheelData wheel_struct_this, float dt_step)
    {

        ////////////////////////////////////////////////////////////////
        // Update wheel local position:
        ////////////////////////////////////////////////////////////////
        
        Vector3 localPos = wheel_struct_this.wheelTransform.localPosition; 
        WheelHit hit;

        if (wheel_struct_this.wheelCollider.GetGroundHit(out hit))
        {
            localPos.y -= Vector3.Dot(wheel_struct_this.wheelTransform.position - hit.point, thisTransform.up) - RADIUS_WHEEL;
            wheel_struct_this.wheelTransform.localPosition = localPos;
        }
        else
            localPos.y = wheel_struct_this.wheelStartPos.y;

        ////////////////////////////////////////////////////////////////
        // Update wheel rotation angle:
        ////////////////////////////////////////////////////////////////
        
        wheel_struct_this.rotation = Mathf.Repeat(
            wheel_struct_this.rotation 
            + dt_step * wheel_struct_this.wheelCollider.rpm * 360.0f / 60.0f, 
            360f);

        ////////////////////////////////////////////////////////////////
        // Update wheel steering angle:
        ////////////////////////////////////////////////////////////////
        
        float angle_steer_deg = wheel_struct_this.wheelCollider.steerAngle;

        // Apply steering angle to wheel transform:
        wheel_struct_this.wheelTransform.localRotation = Quaternion.Euler(
            wheel_struct_this.rotation,
            angle_steer_deg,
            0);
    }

    private void steerHandles()
    {
        handles.transform.localRotation = Quaternion.Euler(
            0, 
            wheel_coll_fwd.steerAngle, 
            0);
    }

    private void RearMudGuardSuspension()
    {
        WheelHit hit;
        if (wheel_coll_back.GetGroundHit(out hit))
            RearMudGuard.transform.rotation = Quaternion.LookRotation(
                thisTransform.position - wheelB.transform.position - RearMudGuardSusOffset, 
                thisTransform.forward);
    }

    void SetWheelFrictionVelocityBased()
    {
        // Set sideways friction with speed gradations:
        if (dt_pos_bike_magn < 10)
            SetWheelFriction(1.5f);
        else if (dt_pos_bike_magn >= 10 && dt_pos_bike_magn < 20)
            SetWheelFriction(2);
        else if (dt_pos_bike_magn >= 20 && dt_pos_bike_magn < 30)
            SetWheelFriction(2.5f);
        else if (dt_pos_bike_magn >= 30 && dt_pos_bike_magn < 40)
            SetWheelFriction(3);
        else
            SetWheelFriction(3.5f);
    }

    void SetWheelFriction(float friction)
    {
        WheelFrictionCurve wfc = wheel_coll_back.sidewaysFriction;

        wfc.stiffness = friction;

        wheel_coll_back.sidewaysFriction = wfc;
        wheel_coll_fwd.sidewaysFriction  = wfc;
    }

    void CalcGear(ref int gear_curr_this, ref float rpm_value_this, float dt_pos_bike_magn_this, bool USE_MOTOR_DISENGAGE)
    {
        const int RANGE_SPEED_GEAR = 13;

        int gear_prev = gear_curr_this;

        gear_curr_this = Mathf.FloorToInt(dt_pos_bike_magn_this / RANGE_SPEED_GEAR);

        if (USE_MOTOR_DISENGAGE || gear_prev <= 2)
            if (gear_curr != gear_prev)
                StartCoroutine(MotorDisengage());

        rpm_value_this = (dt_pos_bike_magn_this % RANGE_SPEED_GEAR) / RANGE_SPEED_GEAR;
    }

    IEnumerator MotorDisengage()
    {
        torque_motor = 0;
        yield return new WaitForSeconds(0.1f);
        torque_motor = TORQUE_MOTOR_MAX;
    }

    TrackCoords GetTrackCoordsRelToBike(Vector3 pos_bike) {
        TrackCoords track_coords;
  
        track_coords.pos_ctrline_near  = Track.instance.GetClosestPointOnCenterLine(pos_bike);
        track_coords.vect_ctrline_tang = Track.instance.GetTangentAtPosition(pos_bike);
        track_coords.ang_ctrline_tang  = FACT_DEG_2_RAD * Track.instance.GetTangentAngleAtPosition(pos_bike);
        track_coords.dist_ctrline_near = Track.instance.GetDistanceAtPosition(pos_bike);

        return track_coords;
    }

    ///////////////////////////////////////////////////////////
    // Ancillary functions:
    //////////////////////////////////////////////////////////
    
    public Vector3 Vector_XZ(Vector3 vect)
    {
        // Project 3D vector on the xz plane:
        Vector3 vect_xz = new Vector3(vect.x, 0f, vect.z);

        return vect_xz;
    }

    public float MagnitudeXZ(Vector3 vect)
    {
        return (float)Math.Sqrt(vect.x*vect.x + vect.z*vect.z);
    }

    public float ConvertSpeedMStoKMH(float speed_mpersec)
    {
        return speed_mpersec * 3.6f;
    }

    ///////////////////////////////////////////////////////////
    // NOTE: made these functions private until we work out real-time issues (22.08.2025):
    //////////////////////////////////////////////////////////

    private Vector3 GetBikePosition()
    {
        return thisTransform.position;
    }
    
    private Vector3 GetBikeDirectionVector()
    {
        return thisTransform.forward;
    }

    private Vector3 GetBikeVelocityVector()
    {
        return rigid_body.velocity;
    }
}
