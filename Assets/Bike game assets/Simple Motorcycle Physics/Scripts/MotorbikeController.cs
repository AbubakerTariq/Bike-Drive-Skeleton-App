using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class MotorbikeController : MonoBehaviour
{
    float FACT_DEG_2_RAD = (float)Math.PI / 180f;

    ////////////////////////////////////////////////////////////////////////////
    // Real-time bike steering step (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // public const int DT_STEP_INPUT_STEER_BIKE_MSEC = 25; // sampling rate for RHB steer input commands 

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering - Manual (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////
    
    const bool USE_STEER_UPDATE_FULL = true; 

    // Steering - Scaling RHB input - BASIC: // TODO: keep or discard
    // const float SCALE_STEER_RHB_MIN  = 0.2f; // for angular deviation of bike's heading wrt to target 
    // const float SCALE_STEER_RHB_BASE = 1.0f;
    const float SCALE_STEER_RHB_MAX  = 1.0f; // make this > 1 to reduce the actual range of RHB rotation   

    // const float SCALE_POS_ROT_START_DEG = 15f;  
    // const float SCALE_POS_ROT_END_DEG   = 30f;

    // Steering - Auto steer - Adjustment parameters 
    // const float ANG_DEV_TARG_REF_DEG = 7.0f; // use together with SCALE_STEER_RHB_MIN

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering - Feedback control (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Preview-ahead time - CRITICAL (26.08.2025):
    public float DT_PREVIEW = 2.0f; //  1.3f; //

    // Gain(s) for tracking target position - CRITICAL
    public float P_GAIN_ERR_POS_TARG = 0.06f; // 0.09f; // 0.045f; //
    public float D_GAIN_ERR_POS_TARG = 0f;
    public float I_GAIN_ERR_POS_TARG = 0f;

    // Gain(s) for tracking reference roll angle - CRITICAL:
    public float P_GAIN_ANGLE_INPUT  = 3.5f; // 
    public float D_GAIN_ANGLE_INPUT  = 0f;
    public float I_GAIN_ANGLE_INPUT  = 0f;

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering - Other:
    ////////////////////////////////////////////////////////////////////////////

    // Factor for steer input - formerly called STEER_INPUT_SENSITIVITY:
    const float FACTOR_STEER_INPUT = 3.5e-2f; // was 23f before factor_steer_bike_speed was removed

    // Steer factor control angle:
    public const float FACTOR_STEER_ANGLE_CTRL = 8.4e-2f; // was 56f before factor_steer_bike_speed was removed

    // Steer factor for control angular speed - higher values stabilize bike:
    const float FACTOR_STEER_DT_ANGLE_CTRL = 12.0e-2f; // was 80f before factor_steer_bike_speed was removed

    // Steer factor for control angular speed squared:
    const float FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER = 2.3f;

    // const float FACTOR_INC_STEER = 20.0f; // 10.0f; // // TODO: keep or discard

    // Return to vertical: scaling factor for roll angular speed
    // const float FACTOR_DT_ANGLE_CTRL_RETURN = 1.25f; // // TODO: keep or discard

    const float FACTOR_ANGLE_WHEEL_FWD = 60f; // 45f; //  75f; //
    const float SPEED_TRANSITION_ANGLE_STEER_BEHAV = 4.0f; // transition speed for wheel steering angle behavior

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Throttle (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Throttle - input geometry settings:
    const float DIST_RADIAL_THROT_FULL_MM = 2.0f; // grippers travel distance for full throttle (mm)

    const float INPUT_THROT_THRESH  = 0.6f; // minimum torque for mobility, also dependent on RADIAL stiffness  
    const float INPUT_THROT_MAX     = 1.3f; // this is a function of RADIAL stiffness  
    const float INPUT_THROT_FBK_MAX = 1.0f;

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Motor torque & acceleration:
    ////////////////////////////////////////////////////////////////////////////
    ///
    const float TORQUE_MOTOR_MAX = 600f; // 500f;

    // Acceleration factor: CRITICAL value - increases top speed but can make turning harder
    const float FACTOR_ACCEL = 2000f; // 1000f; 

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Other:
    ////////////////////////////////////////////////////////////////////////////

    const float RADIUS_WHEEL = 0.7f;

    // Roll and nonslip limit angles:
    public const float ANGLE_ROLL_LOW_DEG         = 42f;
    public const float ANGLE_ROLL_NONSLIP_MAX_DEG = 50f;

    ////////////////////////////////////////////////////////////////////////////
    // Bike states during race (with initial values):
    ////////////////////////////////////////////////////////////////////////////

    public float SPEED_REF_LOW  =  8.0f;
    public float SPEED_REF_HIGH = 25.0f;

    public float rpm_value;
    public Vector3 velocity_rel_collision;

    // Error of preview position wrt target position:
    float err_pos_targ_prev = 0f;
    float int_err_pos_targ = 0f;

    // TARGET bike roll angle / ang vel / integral for steering - CRITICAL:
    private float angle_roll_targ_prev    = 0f;
    private float int_angle_roll_targ     = 0f;

    // ACTUAL bike roll angle / ang vel / integral for steering - CRITICAL:
    private float angle_roll_bike         = 0f;
    private float angle_roll_bike_prev    = 0f;
    private float int_angle_roll_bike     = 0f;

    private float dt_angle_roll_bike      = 0f;
    private float factor_steer_bike_speed = 0f;

    private float torque_motor    = 0f;

    public bool bike_fallen = false;
    public int gear_curr    = 1;

    ////////////////////////////////////////////////////////////////////////////
    // Object instance:
    ////////////////////////////////////////////////////////////////////////////

    public static MotorbikeController instance;

    ////////////////////////////////////////////////////////////////////////////
    // Bike 'physical' parts:
    ////////////////////////////////////////////////////////////////////////////

    // Wheels:
    public WheelCollider wheel_coll_fwd;   
    public WheelCollider wheel_coll_back;  

    public Transform wheelF;
    public Transform wheelB;

    // Handles:
    [SerializeField]  public GameObject handles;

    // Mudguard:
    [SerializeField] public GameObject RearMudGuard;
    public Vector3 RearMudGuardSusOffset;

    private WheelData[] wheels;

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
    public GameObject RagdollAnimation;
    public GameObject Ragdoll;
    
    private bool HardHit;
    private GameObject tempRagdollClone, tempAnimRiderClone;

    ///////////////////////////////////////////////////////////
    // Timers and counters:
    ///////////////////////////////////////////////////////////

    private int step_count_prev = 0;

    /////////////////////////////////////////////////////////// 
    // Data display:
    /////////////////////////////////////////////////////////// 

    private int DT_DISP_DATA_MSEC = 1000;

    ///////////////////////////////////////////////////////////
    // Wheel data class:
    ///////////////////////////////////////////////////////////

    public class WheelData
    {
        public WheelData(Transform transform, WheelCollider collider)
        {
            wheelTransform = transform;
            wheelCollider = collider;
            wheelStartPos = transform.transform.localPosition;
        }

        public Transform wheelTransform;
        public WheelCollider wheelCollider;
        public Vector3 wheelStartPos;
        public float rotation = 0f;
    }

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

    public struct BikePose
    {
        public float angle_roll_bike;
        public float dt_angle_roll_bike;
        public float angle_steer_wheel_fwd;
        public float angle_ctrl;
        public float dt_angle_ctrl;
    }

    /////////////////////////////////////////////////////////////
    // Track coordinates struct:
    /////////////////////////////////////////////////////////////   

    public struct TrackCoords
    {
        public Vector3 pos_ctrline_near;
        public Vector3 vect_ctrline_tang;
        public float curv_ctrline_near;
        public float ang_ctrline_tang;
        public float dist_ctrline_near;
    }

    /////////////////////////////////////////////////////////////
    // Steering input struct:
    /////////////////////////////////////////////////////////////   

    const int N_STEER_UPDATES = 4;

    public struct SteerCalc
    {
        public float[] steer_update;
        public float factor_steer_bike_speed;
        public float steer_term_input ;
        public float steer_term_angle_ctrl ;
        public float steer_term_dt_angle_ctrl;

        public SteerCalc(float[] steer_update_val)
        {
            steer_update = steer_update_val;
            factor_steer_bike_speed  = 0f;
            steer_term_input         = 0f;
            steer_term_angle_ctrl    = 0f;
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
        public float curv_ctrline_preview;
        public float sin_dev_targ;
        public Vector3 vect_ctrline_tang_target;
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

    ////////////////////////////////////////////////////////////////////////////
    // Bike coordinates:
    ////////////////////////////////////////////////////////////////////////////

    static Vector3 NULL_VECTOR3 = Vector3.zero;
    const float NULL_VALUE = 0f;

    private Vector3 pos_bike = NULL_VECTOR3;
    private Vector3 dt_pos_bike = NULL_VECTOR3;
    private Vector3 dir_unit_bike = NULL_VECTOR3;

    /////////////////////////////////////////////////////////////
    // Previous kinematic states:
    /////////////////////////////////////////////////////////////

    private Vector3 pos_bike_prev = new();

    private float angle_ctrl_prev = 0;
    private float dt_angle_ctrl_prev = 0;

    private float dt_pos_bike_magn = 0;
    private float input_steer_prev = 0f;

    /////////////////////////////////////////////////////////////
    // Display settings:
    /////////////////////////////////////////////////////////////

    private bool DISP_MOTOR_CONTROL_ON = true;

    //////////////////////////////////////////////////////////////
    /// Bike speed text:
    //////////////////////////////////////////////////////////////

    [SerializeField] public Text SpeedTxt;

    /////////////////////////////////////////////////////////////
    // METHODS:
    /////////////////////////////////////////////////////////////

    void Awake()
    {
        instance = this;
    }

    void Start()
    {
        // Initialize wheels data:
        wheels = new WheelData[2];
        wheels[0] = new WheelData(wheelF, wheel_coll_fwd);
        wheels[1] = new WheelData(wheelB, wheel_coll_back);

        // Spatial transform and rigid body:
        thisTransform = GetComponent<Transform>();
        rigid_body = GetComponent<Rigidbody>();
        rigid_body.centerOfMass = com;
    }

    void FixedUpdate()
    {
        int step_count = ReHandyBotController.instance.step_count;

        ////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////

        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////
        // Check if bike is balanced:
        ////////////////////////////////////////////////////////////////

        uprightCheck();

        ////////////////////////////////////////////////////////////////
        // If bike is balanced, process control inputs and update bike's kinematic state:
        ////////////////////////////////////////////////////////////////

        if (!bike_fallen)
        {
            ////////////////////////////////////////////////////////////////
            // Bike - local variables:
            ////////////////////////////////////////////////////////////////

            BikeInput bike_input = new();

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

            float pos_radial = ReHandyBotController.instance.distal_data.PositionR;

            if (ReHandyBotController.instance.ExerciseActive)
                bike_input.throttle = InputThrottleCases(pos_radial, MotorbikeController.instance, ReHandyBotController.CASE_CTRL_MODE);
            else
                bike_input.throttle = 0f;

            ////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////
            // BIKE INPUT 3: STEERING
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////   

            ////////////////////////////////////////////////////////////////////////////
            // Compute steering input - feedback based 
            // Updates public DATA VARIABLES for sharing among other classes (for atomicity & real-time updating)
            // NOTE: fbk_ctrl_data contains input_steer_targ
            ////////////////////////////////////////////////////////////////////////////

            float input_steer_targ;

            if (ReHandyBotController.instance.ExerciseActive && MotorbikeController.instance != null && Track.instance != null)
                input_steer_targ = InputSteerTargetFeedback(bike_coords_data, out fbk_ctrl_data);
            else
                input_steer_targ = 0f;

            ////////////////////////////////////////////////////////////////
            // Steering scaling:
            ////////////////////////////////////////////////////////////////            

            float pos_rot = ReHandyBotController.instance.distal_data.PositionP;

            float scale_steer = ScaleInputSteer(pos_rot, MotorbikeController.instance, ReHandyBotController.CASE_CTRL_MODE);

            ////////////////////////////////////////////////////////////////
            // Select steering mode:
            ////////////////////////////////////////////////////////////////

            if (ReHandyBotController.instance.ExerciseActive)
                bike_input.steer_scaled = InputSteerCases(pos_rot, input_steer_targ, scale_steer, ReHandyBotController.CASE_CTRL_MODE);
            else
                bike_input.steer_scaled = 0f;

            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            // BIKE CONTROL ACTIONS: several steps
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////
            // 'Upright force' for zero/low speed balance:
            // NOTE: this is an artificial input for game play purposes, not a realistic condition
            ////////////////////////////////////////////////////////////////

            uprightForce(bike_input.throttle);

            ////////////////////////////////////////////////////////////////
            // Bike control commands - CRITICAL:
            ////////////////////////////////////////////////////////////////            

            MotorbikeControl(bike_input, step_count, out bike_coords_data, out bike_pose_data, ref steer_calc_data);

            ////////////////////////////////////////////////////////////////
            // Adjust torque (key input mode only) and wheel sideways friction:
            ////////////////////////////////////////////////////////////////

            steerHelperTorqueFriction();

            ////////////////////////////////////////////////////////////////
            // Update handles relative angle in transform:
            ////////////////////////////////////////////////////////////////

            // steerHandles();

            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            // Update other public DATA VARIABLES for sharing among other classes (ensures atomicity & real-time updating):
            ////////////////////////////////////////////////////////////////          
            ////////////////////////////////////////////////////////////////

            // Bike input:
            bike_input_data = bike_input;

            // Track coordinates:
            track_coords_data = GetTrackCoordinates(bike_coords_data.pos_bike);

            ////////////////////////////////////////////////////////////////
            // Update text for speed display in Unity (21.08.2025):
            ////////////////////////////////////////////////////////////////       

            // rigid_body.velocity.magnitude shows speed meters per second (m/s):     
            SpeedTxt.text = ConvertSpeedMStoKMH(rigid_body.velocity.magnitude).ToString("F0");
        }

        ////////////////////////////////////////////////////////////////
        // Additional bike dynamics updates (independent of fallen / not fallen):
        ////////////////////////////////////////////////////////////////

        updateWheels();
        RearMudGuardSuspension();
        CalcGear();

        ////////////////////////////////////////////////////////////////
        // Reset after fall:
        ////////////////////////////////////////////////////////////////

        if (Input.GetKey(KeyCode.R) && bike_fallen == true)
            Reset();

        ////////////////////////////////////////////////////////////////
        // Store current step count for any tests:
        ////////////////////////////////////////////////////////////////

        step_count_prev = step_count;

        ////////////////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////////////////

        if (ReHandyBotController.instance.ExerciseActive
            && step_count % (DT_DISP_DATA_MSEC / ReHandyBotController.DT_STEP_APP_MSEC) == 0 
            && DISP_MOTOR_CONTROL_ON)
        {
            // Time elapsed display:
            float timeElapsedValue   = ReHandyBotController.instance.timeElapsedValue;
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
    // Bike Input - Throttle commands - CASES:
    ////////////////////////////////////////////////////////////////  
    
    float InputThrottleCases(float pos_radial, MotorbikeController bike_controller, int case_ctrl_mode)
    {
        float input_throttle;

        float pos_throttle = pos_radial;
        float pos_throttle_zero = ReHandyBotController.POS_RADIAL_BASE_THROT;

        ////////////////////////////////////////////////////////////////
        // Deviation from centerline target (several uses):  
        ////////////////////////////////////////////////////////////////    
        
        float sin_dev_targ = bike_controller.fbk_ctrl_data.sin_dev_targ;

        ////////////////////////////////////////////////////////////////
        // Throttle input from bike trajectory feedback:
        ////////////////////////////////////////////////////////////////
        
        float factor_speed_throttle = 1f - (float)Math.Abs(sin_dev_targ); // was: 1f - sin_dev_targ*sin_dev_targ;
        float input_throttle_fbk = INPUT_THROT_FBK_MAX * factor_speed_throttle;

        ////////////////////////////////////////////////////////////////
        // Manual throttle input - from RHB:
        ////////////////////////////////////////////////////////////////
        
        float SCALE_INPUT_THROTTLE = -1000f / DIST_RADIAL_THROT_FULL_MM;
        
        float input_throttle_manual = Mathf.Clamp(
            SCALE_INPUT_THROTTLE * (pos_throttle - pos_throttle_zero),
            0f, INPUT_THROT_MAX);

        ////////////////////////////////////////////////////////////////
        // Select throttle input:
        ////////////////////////////////////////////////////////////////
        
        switch (case_ctrl_mode)
        {
            case ReHandyBotController.CTRL_ASSISTED:
                input_throttle =
                            ReHandyBotController.FACT_ASSIST_THROTTLE  * input_throttle_fbk
                    + (1f - ReHandyBotController.FACT_ASSIST_THROTTLE) * input_throttle_manual;
                break;

            case ReHandyBotController.CTRL_AUTO_STEER_THROTTLE:

                input_throttle = input_throttle_fbk;
                break;

            case ReHandyBotController.CTRL_AUTO_STEER:
            case ReHandyBotController.CTRL_MANUAL_SIMPLE:

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

    float InputSteerCases(float pos_rot, float input_steer_targ, float scale_steer, int case_ctrl_mode)
    {
        float input_steer_scaled;

        // Steering input - sources: 
        float input_steer_manual = 1.0f / ReHandyBotController.FRAC_POS_ROT_INPUT_USER * pos_rot;

        ////////////////////////////////////////////////////////////////
        // Reference steering input (several cases):
        ////////////////////////////////////////////////////////////////

        float input_steer_ref;

        switch (case_ctrl_mode)
        {
            case ReHandyBotController.CTRL_ASSISTED:

                input_steer_ref =
                           ReHandyBotController.FACT_ASSIST_THROTTLE  * input_steer_targ
                   + (1f - ReHandyBotController.FACT_ASSIST_THROTTLE) * input_steer_manual;
                break;

            case ReHandyBotController.CTRL_AUTO_STEER_THROTTLE:
            case ReHandyBotController.CTRL_AUTO_STEER:

                input_steer_ref = input_steer_targ;
                break;

            case ReHandyBotController.CTRL_MANUAL_SIMPLE:

                input_steer_ref = input_steer_manual;
                break;

            default:

                input_steer_ref = 0f;
                break;
        }

        ////////////////////////////////////////////////////////////////
        // Steering input with scaling factors:
        ////////////////////////////////////////////////////////////////

        input_steer_scaled = scale_steer * input_steer_ref;

        return input_steer_scaled;
    }

    ////////////////////////////////////////////////////////////////
    // Bike steering - Auxiliary functions:
    ////////////////////////////////////////////////////////////////

    float InputSteerTargetFeedback(BikeCoords bike_coords, out FeedbackControl fbk_ctrl)
    {
        // Bike coordinates: 
        Vector3 pos_bike      = bike_coords.pos_bike;
        Vector3 dt_pos_bike   = bike_coords.dt_pos_bike;
        Vector3 dir_unit_bike = bike_coords.dir_unit_bike;

        // Steering input (also included in fbk_ctrl):
        float input_steer_targ = NULL_VALUE;

        Vector3 pos_preview        = NULL_VECTOR3;
        Vector3 pos_track_targ     = NULL_VECTOR3;
        float angle_roll_targ      = NULL_VALUE;
        float dt_angle_roll_targ   = NULL_VALUE;
        float curv_ctrline_preview = NULL_VALUE;
        float sin_dev_targ         = NULL_VALUE; // angular deviation of bike's heading wrt to target
        Vector3 vect_ctrline_tangent_targ = NULL_VECTOR3;

        // Deviation wrt nearest point on track: // TODO: remove at a later date
        /*
        Vector3 err_pos_near        = NULL_VECTOR3;
        Vector3 vect_unit_turn_near = NULL_VECTOR3; 
                Vector3 vect_bike_to_targ   = NULL_VECTOR3;
        */

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
            out curv_ctrline_preview, 
            out vect_ctrline_tangent_targ);

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 2: lateral displacement control - roll angle /ang vel target:
        ////////////////////////////////////////////////////////////////////////////
        
        // Deviation wrt target point on track:
        Vector3 err_pos_targ_vect = pos_track_targ - pos_preview;
        Vector3 vect_unit_turn_targ = Vector3.Cross(dir_unit_bike, err_pos_targ_vect.normalized); // test vector to establish turn direction
        float sgn_turn_targ = (float)Math.Sign(-vect_unit_turn_targ.y);

        // Error of preview position wrt target position:
        float err_pos_targ = sgn_turn_targ * err_pos_targ_vect.magnitude;

        // Error of preview position: time derivative:
        float dt_err_pos_targ = (err_pos_targ - err_pos_targ_prev) / dt_step;

        // Error of preview position: integral
        int_err_pos_targ += err_pos_targ * dt_step;

        // Store error for next iteration:
        err_pos_targ_prev = err_pos_targ;

        // Target roll angle:
        angle_roll_targ =
            P_GAIN_ERR_POS_TARG   * err_pos_targ
            + D_GAIN_ERR_POS_TARG * dt_err_pos_targ
            + I_GAIN_ERR_POS_TARG * int_err_pos_targ;

        // Target roll angular velocity:
        dt_angle_roll_targ = (angle_roll_targ - angle_roll_targ_prev) / dt_step;

        // Target roll angle integral:
        int_angle_roll_targ += angle_roll_targ * dt_step;

        // Store roll angle for next iteration:
        angle_roll_targ_prev = angle_roll_targ;

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 3: steering input - KEY STEP
        ////////////////////////////////////////////////////////////////////////////

        input_steer_targ = 
            P_GAIN_ANGLE_INPUT   * (    angle_roll_targ -     angle_roll_bike) 
            + D_GAIN_ANGLE_INPUT * ( dt_angle_roll_targ -  dt_angle_roll_bike)
            + I_GAIN_ANGLE_INPUT * (int_angle_roll_targ - int_angle_roll_bike); 

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 4: angular deviation of bike's heading wrt to target tangent:
        ////////////////////////////////////////////////////////////////////////////  

        Vector3 vect_unit_dev_tangent = Vector3.Cross(
            dir_unit_bike, vect_ctrline_tangent_targ.normalized);

        sin_dev_targ = vect_unit_dev_tangent.y;

        ////////////////////////////////////////////////////////////////
        // Update data variables' struct for sharing among other classes (for atomicity & real-time updating):
        ////////////////////////////////////////////////////////////////    

        fbk_ctrl.pos_preview = pos_preview;
        fbk_ctrl.pos_track_targ = pos_track_targ;
        fbk_ctrl.angle_roll_targ = angle_roll_targ;
        fbk_ctrl.dt_angle_roll_targ = dt_angle_roll_targ;
        fbk_ctrl.input_steer_targ = input_steer_targ;
        fbk_ctrl.curv_ctrline_preview = curv_ctrline_preview;
        fbk_ctrl.sin_dev_targ = sin_dev_targ;
        fbk_ctrl.vect_ctrline_tang_target = vect_ctrline_tangent_targ;

        return input_steer_targ;
    }

    float ScaleInputSteer(float pos_rot, MotorbikeController bike_controller, int case_ctrl_mode)
    {
        // Steering scale:
        float scale_steer;

        // RHB input:
        // float pos_rot_abs = (float)Math.Abs(pos_rot);

        // Reference angles:
        // float pos_rot_start = FACT_DEG_2_RAD * SCALE_POS_ROT_START_DEG;
        // float pos_rot_end   = FACT_DEG_2_RAD * SCALE_POS_ROT_END_DEG;

        ////////////////////////////////////////////////////////////////
        // Steer scaling: adjustments for auto steer (TODO: keep or discard):
        ////////////////////////////////////////////////////////////////

        // TODO: keep or discard
        /*      
        float scale_steer_base_adj;

        // Scale for RHB rotational input: adjust minimum scale value for angular deviation of bike's heading wrt to target:
        float sin_dev_targ     = bike_controller.fbk_ctrl_data.sin_dev_targ;
        float sin_dev_targ_ref = (float)Math.Sin(FACT_DEG_2_RAD * ANG_DEV_TARG_REF_DEG);

        if ((  case_ctrl_mode == ReHandyBotController.CTRL_AUTO_STEER_THROTTLE
            || case_ctrl_mode == ReHandyBotController.CTRL_AUTO_STEER)
            && sin_dev_targ <= sin_dev_targ_ref)
        {
            scale_steer_base_adj = 
                (SCALE_STEER_RHB_BASE - SCALE_STEER_RHB_MIN)
                / sin_dev_targ_ref
                * (float)Math.Abs(sin_dev_targ) + SCALE_STEER_RHB_MIN;
        }
        else
            scale_steer_base_adj = SCALE_STEER_RHB_BASE;
        */

        ////////////////////////////////////////////////////////////////
        // Calculate scale for RHB rotational input:
        ////////////////////////////////////////////////////////////////

        // TODO: keep or discard
        /*
        if (pos_rot_abs < pos_rot_start)
            scale_steer = scale_steer_base_adj;

        else if (pos_rot_abs > pos_rot_end)
            scale_steer = SCALE_STEER_RHB_MAX;

        else
            scale_steer =
                (pos_rot_abs - pos_rot_start) / (pos_rot_end - pos_rot_start)
                * (SCALE_STEER_RHB_MAX - scale_steer_base_adj)
                + scale_steer_base_adj;
        */

        scale_steer = SCALE_STEER_RHB_MAX;

        return scale_steer;
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
        dt_pos_bike_magn = dt_pos_bike.magnitude;

        ////////////////////////////////////////////////////////////////
        // Bike roll angle (rad):
        ////////////////////////////////////////////////////////////////

        if (transform.eulerAngles.z > 180)
            angle_roll_bike = FACT_DEG_2_RAD * transform.eulerAngles.z - 2f * (float)Math.PI;
        else
            angle_roll_bike = FACT_DEG_2_RAD * transform.eulerAngles.z;

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
        // "Control" angle and angular vel (see Simple Motorcycle Physics, p. 9):
        ////////////////////////////////////////////////////////////////

        // TODO: keep or discard
        /*
        float angle_ctrl = Vector3.Dot(dir_unit_bike, Vector3.Cross(thisTransform.up, new Vector3(0, 1, 0)));
        float dt_angle_ctrl = (angle_ctrl - angle_ctrl_prev) / dt_step;

        angle_ctrl_prev = angle_ctrl;
        dt_angle_ctrl_prev = dt_angle_ctrl;
        */

        float angle_ctrl    = angle_roll_bike;
        float dt_angle_ctrl = dt_angle_roll_bike;

        ////////////////////////////////////////////////////////////////
        // Track the multiple updates to bike_input.steer that happen in this function:
        ////////////////////////////////////////////////////////////////

        float[] steer_update = new float[N_STEER_UPDATES + 1];

        ////////////////////////////////////////////////////////////////
        // Initialize bike_input.steer updates:
        ////////////////////////////////////////////////////////////////

        steer_update[0] = bike_input.steer_scaled;

        if (USE_STEER_UPDATE_FULL)
        {
            ////////////////////////////////////////////////////////////////
            // bike_input.steer UPDATE 1 (update angle_ctrl, dt_angle_ctrl):
            ////////////////////////////////////////////////////////////////

            float ratio_speed;

            // Low-speed case (steer update):
            if (dt_pos_bike_magn < SPEED_REF_LOW)
            {
                ratio_speed = dt_pos_bike_magn / SPEED_REF_LOW;

                angle_ctrl    *= (2.0f - ratio_speed);
                dt_angle_ctrl *= ratio_speed * ratio_speed;

                bike_input.acceleration += 3.0f * Mathf.Abs(angle_ctrl) * (1.0f - ratio_speed);
                bike_input.steer_scaled *= ratio_speed *ratio_speed;
            }

            // High-speed case (NO steer update):
            else if (dt_pos_bike_magn > SPEED_REF_HIGH)
            {
                ratio_speed = dt_pos_bike_magn / SPEED_REF_HIGH;

                // Adjust roll angular speed for return to upright:
                if (dt_angle_ctrl * angle_ctrl < 0f)
                    dt_angle_ctrl *= ratio_speed; // was FACTOR_DT_ANGLE_CTRL_RETURN 
            }

            steer_update[1] = bike_input.steer_scaled;

            ////////////////////////////////////////////////////////////////
            // bike_input.steer UPDATE 2 (update steer input with angle_ctrl squared):   
            ////////////////////////////////////////////////////////////////         

            steer_update[2] = bike_input.steer_scaled * (1 - FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER * (float)Math.Pow(angle_ctrl, 2));

            bike_input.steer_scaled = steer_update[2];
        }
        else
        {
            steer_update[1] = bike_input.steer_scaled;
            steer_update[2] = bike_input.steer_scaled;
        }

        ////////////////////////////////////////////////////////////////
        // bike_input.steer UPDATE 3 (steer terms weighted sum: input, angle_ctrl, dt_angle_ctrl):
        ////////////////////////////////////////////////////////////////

        // Bike speed factor - update: // TODO: keep or discard
        // factor_steer_bike_speed = 1.5e-3f; // 1f / (1f + (float)Math.Pow(dt_pos_bike_magn, 2));

        float steer_term_input         = FACTOR_STEER_INPUT          * bike_input.steer_scaled;
        float steer_term_angle_ctrl    = FACTOR_STEER_ANGLE_CTRL     * angle_ctrl;
        float steer_term_dt_angle_ctrl = FACTOR_STEER_DT_ANGLE_CTRL  * dt_angle_ctrl;

        steer_update[3] = steer_term_input - steer_term_angle_ctrl - steer_term_dt_angle_ctrl; // was factor_steer_bike_speed *

        bike_input.steer_scaled = steer_update[3];

        ////////////////////////////////////////////////////////////////
        // bike_input.steer UPDATE 4 (clamp with input_steer_prev):
        ////////////////////////////////////////////////////////////////

        // TODO: keep or discard
        // float inc_steer = FACTOR_INC_STEER * dt_step;
        // steer_update[4] = Mathf.Clamp(bike_input.steer, input_steer_prev - inc_steer, input_steer_prev + inc_steer);

        steer_update[4] = steer_update[3];

        bike_input.steer_scaled = steer_update[4];

        ////////////////////////////////////////////////////////////////
        // Save steering value for next step:
        ////////////////////////////////////////////////////////////////

        input_steer_prev = bike_input.steer_scaled;

        ////////////////////////////////////////////////////////////////
        // Update wheels steering angle (wheel colliders) - CRITICAL:
        ////////////////////////////////////////////////////////////////

        if (dt_pos_bike_magn > SPEED_TRANSITION_ANGLE_STEER_BEHAV)
            wheel_coll_fwd.steerAngle = FACTOR_ANGLE_WHEEL_FWD * bike_input.steer_scaled; // Mathf.Clamp(bike_input.steer_scaled, -1.0f, 1.0f) * FACTOR_ANGLE_FRONT_WHEEL;
        else
            wheel_coll_fwd.steerAngle = Mathf.Clamp(bike_input.steer_scaled, -dt_pos_bike_magn, dt_pos_bike_magn); // TODO: how come there is no scaling

        ////////////////////////////////////////////////////////////////
        // Apply input for torque & force control - CTRITICAL:
        ////////////////////////////////////////////////////////////////

        wheel_coll_back.motorTorque = torque_motor * bike_input.throttle;

        float scale_factor_accel = 0f;

        if (dt_pos_bike_magn < SPEED_REF_HIGH)
            scale_factor_accel = 1f;
        else
            scale_factor_accel = 0.5f;

        rigid_body.AddForce(scale_factor_accel * FACTOR_ACCEL * bike_input.throttle * transform.forward);

        ////////////////////////////////////////////////////////////////
        // Update rigid-body Cartesian velocities:
        ////////////////////////////////////////////////////////////////

        if (Input.GetAxis("Vertical") < 0)
            rigid_body.velocity = new Vector3(
                rigid_body.velocity.x,
                rigid_body.velocity.y,
                rigid_body.velocity.z);

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
        bike_pose.angle_steer_wheel_fwd = wheel_coll_fwd.steerAngle * (float)Math.PI / 180f;
        bike_pose.angle_ctrl = angle_ctrl;
        bike_pose.dt_angle_ctrl = dt_angle_ctrl;

        ////////////////////////////////////////////////////////////////
        // Steer calculation data:  
        ////////////////////////////////////////////////////////////////

        for (int i = 0; i <= N_STEER_UPDATES; i++)
            steer_calc.steer_update[i] = steer_update[i];

        steer_calc.factor_steer_bike_speed = factor_steer_bike_speed;
        steer_calc.steer_term_input = steer_term_input;
        steer_calc.steer_term_angle_ctrl = steer_term_angle_ctrl;
        steer_calc.steer_term_dt_angle_ctrl = steer_term_dt_angle_ctrl;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

        if (step_count % (DT_DISP_DATA_MSEC / ReHandyBotController.DT_STEP_APP_MSEC) == 0 && step_count > step_count_prev &&
            ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            ExternalConsoleLogger.Log("    factor_speed_steer [" + String.Format("{0:#0.000}", factor_steer_bike_speed) + "]\n");
            ExternalConsoleLogger.Log("    steer_term_input   [" + String.Format("{0:#0.000}", steer_term_input) + "]");
            ExternalConsoleLogger.Log("    steer_term_angle   [" + String.Format("{0:#0.000}", steer_term_angle_ctrl) + "]");
            ExternalConsoleLogger.Log("    steer_term_dt_angle[" + String.Format("{0:#0.000}", steer_term_dt_angle_ctrl) + "]");

            // ExternalConsoleLogger.Log("    factor_steer_input         [" + FACTOR_STEER_INPUT + "]");
            // ExternalConsoleLogger.Log("    factor_steer_angle_ctrl    [" + FACTOR_STEER_ANGLE_CTRL_REF + "]");
            // ExternalConsoleLogger.Log("    FACTOR_STEER_DT_ANGLE_CTRL [" + FACTOR_STEER_DT_ANGLE_CTRL + "]");

            ExternalConsoleLogger.Log("\n");

            // for (int j = 0; j <= N_STEER_UPDATES; j++)
            //     ExternalConsoleLogger.Log("    steer_update[" + j + "] = " + String.Format("{0:#0.000}", steer_update[j]));
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Data for feedback (assisted / auto-steer) control:
    ////////////////////////////////////////////////////////////////////////////

    private void TrackDataFeedbackControl(
        Vector3 pos_bike, Vector3 dt_pos_bike, Vector3 dir_unit_bike,
        float dt_preview, Track track_this,
        out Vector3 pos_preview_this, 
        out Vector3 pos_track_targ_this, // ref Vector3 pos_track_near_this,
        out float curv_ctrline_preview_this, 
        out Vector3 vect_ctrline_tang_target_this)
    {
        // Obtain preview point:
        float dist_preview = dt_pos_bike.magnitude * dt_preview; // distance to preview point ahead
        pos_preview_this = pos_bike + dist_preview * dir_unit_bike;

        // Obtain target point on track centerline:
        pos_track_targ_this = track_this.GetClosestPointOnCenterLine(pos_preview_this);

        // Curvature of centerline at preview point:
        curv_ctrline_preview_this = track_this.GetCurvatureAtPosition(pos_track_targ_this);

        // Tangent vector at target point:
        vect_ctrline_tang_target_this = track_this.GetTangentAtPosition(pos_preview_this);
    }

    ////////////////////////////////////////////////////////////////
    // Ancillary functions:
    ////////////////////////////////////////////////////////////////

    private void Reset()
    {
        // Reset bike position to the closest center point to the bike's current position
        Transform t = GetComponent<Transform>();
        t.position = Track.instance.GetClosestPointOnCenterLine(t.position) + new Vector3(0f, 0.1f, 0f);

        // Reset bike rotation to align with the rotation of the track
        Quaternion rotation = Track.instance.GetTrackRotationAtPosition(t.position);
        float yaw = rotation.eulerAngles.y;
        t.transform.rotation = Quaternion.Euler(0f, yaw, 0f);

        // Reset the bike velocity to 0 (optional)
        rigid_body.velocity = Vector3.zero;
        rigid_body.angularDrag = 100;
        rigid_body.centerOfMass = com;

        HardHit = false;
        bike_fallen = false;
        Destroy(tempRagdollClone);
        Destroy(tempAnimRiderClone);
        Rider.SetActive(true);
    }

    private void uprightForce(float input_throttle)
    {
        bool input_force_trq_on;

        if (input_throttle >= INPUT_THROT_THRESH)
            input_force_trq_on = true;
        else
            input_force_trq_on = false;

        // rigid_body.angularDrag -= 100f * Time.deltaTime;
        // rigid_body.angularDrag = Mathf.Clamp(rigid_body.angularDrag, 0.1f, 100f);

        if (dt_pos_bike_magn < SPEED_TRANSITION_ANGLE_STEER_BEHAV && !input_force_trq_on) // Input.GetKey(KeyCode.W) (11.06.2025)
        {
            transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, 0);
            rigid_body.constraints = RigidbodyConstraints.FreezeAll;
        }
        else
            rigid_body.constraints = RigidbodyConstraints.None;
    }

    void OnCollisionEnter(Collision collision)
    {
        velocity_rel_collision = collision.relativeVelocity;
        if (collision.relativeVelocity.magnitude > 30)
            HardHit = true;
    }

    public void uprightCheck()
    {
        float angle_roll_nonslip_max = FACT_DEG_2_RAD * ANGLE_ROLL_NONSLIP_MAX_DEG;

        if ((Mathf.Abs(angle_roll_bike) > angle_roll_nonslip_max || Input.GetKeyDown(KeyCode.F) || HardHit == true)
            && bike_fallen == false)
        {
            Rider.SetActive(false);
            tempRagdollClone = Instantiate(Ragdoll);
            tempAnimRiderClone = Instantiate(RagdollAnimation);
            rigid_body.centerOfMass = new Vector3(0, 0.5f, 0);
            bike_fallen = true;
        }
    }

    private void updateWheels()
    {
        float dt_step = Time.fixedDeltaTime;

        foreach (WheelData wheel_this in wheels)
        {
            WheelHit hit;

            Vector3 localPos = wheel_this.wheelTransform.localPosition;
            if (wheel_this.wheelCollider.GetGroundHit(out hit))
            {
                localPos.y -= Vector3.Dot(wheel_this.wheelTransform.position - hit.point, transform.up) - RADIUS_WHEEL;
                wheel_this.wheelTransform.localPosition = localPos;
            }
            else
                localPos.y = wheel_this.wheelStartPos.y;

            wheel_this.rotation = Mathf.Repeat(wheel_this.rotation + dt_step * wheel_this.wheelCollider.rpm * 360.0f / 60.0f, 360f);
            wheel_this.wheelTransform.localRotation = Quaternion.Euler(
                wheel_this.rotation,
                Mathf.Lerp(wheel_this.wheelTransform.localRotation.y, wheel_this.wheelCollider.steerAngle, Time.deltaTime * 10),
                0);
        }
    }

    private void steerHandles()
    {
        handles.transform.localRotation = 
            Quaternion.Euler(0, Mathf.Lerp(handles.transform.localRotation.y, wheel_coll_fwd.steerAngle, Time.deltaTime * 10), 0);
    }

    private void RearMudGuardSuspension()
    {
        WheelHit hit;
        if (wheel_coll_back.GetGroundHit(out hit))
            RearMudGuard.transform.rotation = Quaternion.LookRotation(transform.position - wheelB.transform.position - RearMudGuardSusOffset, transform.forward);
    }

    void steerHelperTorqueFriction()
    {
        // Set sideways friction with speed gradations:
        if (dt_pos_bike_magn < 10)
            SetWheelFriction(1.5f);
        else if (dt_pos_bike_magn < 20 && dt_pos_bike_magn > 10)
            SetWheelFriction(2);
        else if (dt_pos_bike_magn < 30 && dt_pos_bike_magn > 20)
            SetWheelFriction(2.5f);
        else if (dt_pos_bike_magn < 40 && dt_pos_bike_magn > 20)
            SetWheelFriction(3);
        else
            SetWheelFriction(3.5f);
    }

    void SetWheelFriction(float friction)
    {
        WheelFrictionCurve wfc;
        wfc = wheel_coll_back.sidewaysFriction;
        wfc.stiffness = friction;
        wheel_coll_back.sidewaysFriction = wfc;
        wheel_coll_fwd.sidewaysFriction = wfc;
    }

    void CalcGear()
    {
        int FACT_GEAR = 13;
        var gear_prev = gear_curr;

        gear_curr = Mathf.FloorToInt(dt_pos_bike_magn / FACT_GEAR);

        if (gear_curr != gear_prev)
            StartCoroutine(MotorDisengage());

        rpm_value = (dt_pos_bike_magn % FACT_GEAR) / FACT_GEAR;
    }

    IEnumerator MotorDisengage()
    {
        torque_motor = 0;
        yield return new WaitForSeconds(0.1f);
        torque_motor = TORQUE_MOTOR_MAX;
    }

    TrackCoords GetTrackCoordinates(Vector3 pos_bike) {
        TrackCoords track_coords;
  
        track_coords.pos_ctrline_near = Track.instance.GetClosestPointOnCenterLine(pos_bike);
        track_coords.vect_ctrline_tang = Track.instance.GetTangentAtPosition(pos_bike);
        track_coords.curv_ctrline_near = Track.instance.GetCurvatureAtPosition(pos_bike);
        track_coords.ang_ctrline_tang = (float) Math.PI / 180f * Track.instance.GetTangentAngleAtPosition(pos_bike);
        track_coords.dist_ctrline_near = Track.instance.GetDistanceAtPosition(pos_bike);

        return track_coords;
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
        return transform.position;
    }
    
    private Vector3 GetBikeDirectionVector()
    {
        return transform.forward;
    }

    private Vector3 GetBikeVelocityVector()
    {
        return rigid_body.velocity;
    }
}
