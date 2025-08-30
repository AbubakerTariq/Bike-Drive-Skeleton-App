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

    public const int DT_STEP_INPUT_STEER_BIKE_MSEC = 25; // sampling rate for RHB steer input commands 

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////
    
    const bool USE_STEER_UPDATE_FULL = true; // see motoControlRHB()

    // Steering - input settings:
    const float FACT_STEER_RESPONSE  = 2.3f; // 4.0f; // 2.0f; // factor affecting steering response (the higher the larger the response)

    // Steering - Scaling RHB input - BASIC:
    const float SCALE_STEER_RHB_MIN  = 0.2f; // for angular deviation of bike's heading wrt to target:
    const float SCALE_STEER_RHB_BASE = 1.0f;
    const float SCALE_STEER_RHB_MAX  = 2.0f; // make this > 1 to reduce the actual range of RHB rotation  

    const float SCALE_POS_ROT_START_DEG = 15f; // TODO: test 10f for straight segments 
    const float SCALE_POS_ROT_END_DEG   = 30f;

    // Steering - Auto steer - Adjustment parameters 
    const float ANG_DEV_TARG_REF_DEG = 7.0f; // use together with SCALE_STEER_RHB_MIN

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Steering - Other:
    ////////////////////////////////////////////////////////////////////////////

    // Factor for steer input - formerly called STEER_INPUT_SENSITIVITY:
    const float FACTOR_STEER_INPUT = 10f; // 30f; // 45f; // 

    // Steer factor control angle:
    const float FACTOR_STEER_ANGLE_CTRL_REF = 56f; // based on range of 48 to 65 deg in original code

    // Steer factor for control angular speed - higher values stabilize bike:
    // Link it with P_GAIN_ANGLE_INPUT_BIKE in ReHandyBotController (26.08.2025):
    const float FACTOR_STEER_DT_ANGLE_CTRL = 80f; // 30f; //

    // Steer factor for control angular speed squared:
    const float FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER = 2.3f;

    const float FACTOR_INC_STEER = 20.0f; // 10.0f;

    // Return to vertical: scaling factor for roll angular speed
    const float FACTOR_DT_ANGLE_CTRL_RETURN = 1.25f;

    const float ANGLE_STEER_FRONT_WHEEL_MAX_DEG = 60f; // 75f; // 45f;
    const float SPEED_TRANSITION_ANGLE_STEER_BEHAV = 1.0f; // transition speed for wheel steering angle behavior

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - Throttle (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Throttle input:
    const bool USE_RHB_THROTTLE = true;

    // Throttle - input geometry settings:
    const float DIST_RADIAL_THROT_FULL_MM = 2.0f; // grippers travel distance for full throttle (mm)

    const float INPUT_THROT_THRESH = 0.6f; // minimum torque for mobility, also dependent on RADIAL stiffness  
    const float INPUT_THROT_MAX = 1.3f; // this is a function of RADIAL stiffness  
    const float INPUT_THROT_AUTO_STEER_MAX = 1.0f;

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

    [HideInInspector] public float factor_steer_angle_ctrl = FACTOR_STEER_ANGLE_CTRL_REF;

    // TODO; verify there are no problems with Inspector (26.08.2025):
    [HideInInspector] public float SPEED_REF_LOW  =  8.0f;
    [HideInInspector] public float SPEED_REF_HIGH = 25.0f;

    [HideInInspector] public float rpm_value;
    [HideInInspector] public Vector3 velocity_rel_collision;

    // Bike roll angle - CRITICAL for steering:
    private float angle_roll_bike      = 0f;
    private float angle_roll_bike_prev = 0f;

    private float factor_steer_input = FACTOR_STEER_INPUT;
    private float factor_steer_bike_speed = 0f;

    private float torque_motor    = 0f;

    public bool bike_fallen = false;
    public int gear_curr = 1;

    ////////////////////////////////////////////////////////////////////////////
    // Control inputs from RHB (30.08.2025):
    ////////////////////////////////////////////////////////////////////////////

    // Reference roll angle from RHB
    // Based on end-effector rotation angle - CRITICAL for steering:
    private float angle_roll_rhb = 0f;

    ////////////////////////////////////////////////////////////////////////////
    // Object instance:
    ////////////////////////////////////////////////////////////////////////////

    public static MotorbikeController instance;

    ////////////////////////////////////////////////////////////////////////////
    // Bike 'physical' parts:
    ////////////////////////////////////////////////////////////////////////////

    // Wheels:
    public WheelCollider wheel_coll_fwd; // gameObject.AddComponent<WheelCollider>();
    public WheelCollider wheel_coll_back; // gameObject.AddComponent<WheelCollider>();

    public Transform wheelF;
    public Transform wheelB;

    // Handles:
    [SerializeField]
    public GameObject handles;

    // Mudguard:
    [SerializeField]
    public GameObject RearMudGuard;
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
    bool HardHit;
    GameObject tempRagdollClone, tempAnimRiderClone;

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

    public struct BikeInputRHB
    {
        public float steer;
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

    /////////////////////////////////////////////////////////////
    // Public DATA VARIABLES for sharing data among classes:
    /////////////////////////////////////////////////////////////   

    public BikeCoords bike_coords_data = new(); // Motorbike coordinates
    public BikeInputRHB bike_input_rhb_data = new(); // Motorbike input
    public BikePose bike_pose_data = new(); // Motorbike pose
    public TrackCoords track_coords_data = new(); // Track coordinates
    public SteerCalc steer_calc_data = new SteerCalc(new float[N_STEER_UPDATES + 1]); // steer input computations

    /////////////////////////////////////////////////////////////
    // Previous kinematic states:
    /////////////////////////////////////////////////////////////

    private Vector3 pos_bike_prev = new();

    private float angle_ctrl_prev = 0;
    private float dt_angle_ctrl_prev = 0;

    private float dt_pos_bike_magn = 0;
    private float input_steer_prev = 0f;

    ///////////////////////////////////////////////////////////
    // Input variables:
    ///////////////////////////////////////////////////////////
       
    // Steering mode:
    const int STEER_MODE_MANUAL       = 1;
    const int STEER_MODE_ASSISTED_TRACKING = 2;
    const int STEER_MODE_KEYB         = 3;

    const int CASE_STEER_MODE = STEER_MODE_ASSISTED_TRACKING;

    /////////////////////////////////////////////////////////////
    // Display settings:
    /////////////////////////////////////////////////////////////

    private bool DISP_FIXED_UPDATE_ON = false;
    private bool DISP_MOTOR_CONTROL_ON = true;

    //////////////////////////////////////////////////////////////
    /// Bike speed text:
    //////////////////////////////////////////////////////////////

    [SerializeField] public Text SpeedTxt;

    /////////////////////////////////////////////////////////////
    // METHODS:
    /////////////////////////////////////////////////////////////

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

            BikeInputRHB bike_input_rhb = new();

            ////////////////////////////////////////////////////////////////
            // Acceleration input - KEYBOARD:
            ////////////////////////////////////////////////////////////////         

            if (USE_RHB_THROTTLE)
                bike_input_rhb.acceleration = 0f;
            
            else if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
                bike_input_rhb.acceleration = 1f; // this input gets modified at low speeds - see motorControl()

            ////////////////////////////////////////////////////////////////
            // Deviation from centerline target (several uses):
            ////////////////////////////////////////////////////////////////
           
            float sin_dev_targ = ReHandyBotController.instance.auto_steer_ctrl_data.sin_dev_targ;

            ////////////////////////////////////////////////////////////////
            // RHB radial input - throttle:
            ////////////////////////////////////////////////////////////////

            float pos_throttle      = ReHandyBotController.instance.distal_data.PositionR;
            float pos_throttle_zero = ReHandyBotController.instance.POS_RADIAL_BASE_THROT;

            if (ReHandyBotController.instance.ExerciseActive)
            {
                if (ReHandyBotController.AUTO_THROTTLE_RHB_ON)
                {
                    float factor_speed_throttle = 1f - (float)Math.Abs(sin_dev_targ); // 1f - sin_dev_targ * sin_dev_targ;

                    bike_input_rhb.throttle = factor_speed_throttle * INPUT_THROT_AUTO_STEER_MAX;
                }
                else
                    bike_input_rhb.throttle = Mathf.Clamp(-1000f / DIST_RADIAL_THROT_FULL_MM * (pos_throttle - pos_throttle_zero),
                        0f, INPUT_THROT_MAX);
            }
            else
                bike_input_rhb.throttle = 0f;

            ////////////////////////////////////////////////////////////////
            // RHB rotational input - convert to RAW steering input:
            ////////////////////////////////////////////////////////////////

            // RHB input - CRITICAL:
            float pos_rot = ReHandyBotController.instance.distal_data.PositionP;
            float pos_rot_abs = (float)Math.Abs(pos_rot);

            // Reference angles:
            float pos_rot_start = FACT_DEG_2_RAD * SCALE_POS_ROT_START_DEG;
            float pos_rot_end   = FACT_DEG_2_RAD * SCALE_POS_ROT_END_DEG;

            // Scale for RHB rotational input: adjust minimum scale value for angular deviation of bike's heading wrt to target:
            float sin_dev_targ_ref = (float)Math.Sin(FACT_DEG_2_RAD * ANG_DEV_TARG_REF_DEG);

            float scale_steer_rhb_base_adj;

            if (sin_dev_targ <= sin_dev_targ_ref)
                scale_steer_rhb_base_adj = (SCALE_STEER_RHB_BASE - SCALE_STEER_RHB_MIN) / sin_dev_targ_ref 
                    * (float)Math.Abs(sin_dev_targ) + SCALE_STEER_RHB_MIN;
            else
                scale_steer_rhb_base_adj = SCALE_STEER_RHB_BASE;

            // Calculate scale for RHB rotational input:
            float scale_steer_rhb;

            if (pos_rot_abs < pos_rot_start)
                scale_steer_rhb = scale_steer_rhb_base_adj;
            else if (pos_rot_abs > pos_rot_end)
                scale_steer_rhb = SCALE_STEER_RHB_MAX;
            else
                scale_steer_rhb =
                    (pos_rot_abs - pos_rot_start) / (pos_rot_end - pos_rot_start)
                    * (SCALE_STEER_RHB_MAX - scale_steer_rhb_base_adj) 
                    + scale_steer_rhb_base_adj;

            ////////////////////////////////////////////////////////////////
            // Select steering mode:
            ////////////////////////////////////////////////////////////////

            // Steering input: MANUAL / TRACKING CONTROL:
            if (CASE_STEER_MODE == STEER_MODE_MANUAL || CASE_STEER_MODE == STEER_MODE_ASSISTED_TRACKING)
            {
                if (ReHandyBotController.instance.ExerciseActive)
                {
                    float input_steer_raw;

                    // Select position value:
                    if (CASE_STEER_MODE == STEER_MODE_ASSISTED_TRACKING)
                        input_steer_raw = ReHandyBotController.instance.auto_steer_ctrl_data.input_steer_fbk;
                    else // MANUAL mode
                        input_steer_raw = pos_rot;  

                    // Angle input with proportionality factor:   
                    bike_input_rhb.steer = FACT_STEER_RESPONSE * scale_steer_rhb * input_steer_raw;
                }
                else
                    bike_input_rhb.steer = 0f;
            }
                
            // Steering input - KEYBOARD:
            /*
            else if (CASE_STEER_MODE == STEER_MODE_KEYB)
            {
                if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
                    bike_input_rhb.steer = 1;
                else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
                    bike_input_rhb.steer = -1;
            }
            */

            ////////////////////////////////////////////////////////////////
            // 'Upright force' calculations:
            ////////////////////////////////////////////////////////////////

            bool input_force_trq_on;

            if (USE_RHB_THROTTLE)
            {
                if (bike_input_rhb.throttle >= INPUT_THROT_THRESH)
                    input_force_trq_on = true;
                else
                    input_force_trq_on = false;
            }
            else
            {
                if (bike_input_rhb.acceleration > 0f)
                    input_force_trq_on = true;
                else
                    input_force_trq_on = false;
            }

            uprightForce(input_force_trq_on);

             ////////////////////////////////////////////////////////////////
            // Bike control commands (CRITICAL):
            ////////////////////////////////////////////////////////////////            

            motoControlRHB(bike_input_rhb, step_count, out bike_coords_data, out bike_pose_data, ref steer_calc_data);

            ////////////////////////////////////////////////////////////////
            // Adjust torque (key input mode only) and wheel sideways friction:
            ////////////////////////////////////////////////////////////////

            steerHelperTorqueFriction();

            ////////////////////////////////////////////////////////////////
            // Update handles relative angle in transform:
            ////////////////////////////////////////////////////////////////

            steerHandles();

            ////////////////////////////////////////////////////////////////
            // Update other public DATA VARIABLES for sharing among other classes (ensures atomicity & real-time updating):
            ////////////////////////////////////////////////////////////////            

            // Bike input:
            bike_input_rhb_data = bike_input_rhb;

            // Track coordinates:
            track_coords_data = GetTrackCoordinates(bike_coords_data.pos_bike);

            ////////////////////////////////////////////////////////////////
            // Update text for speed display in Unity (21.08.2025):
            ////////////////////////////////////////////////////////////////       

            SpeedTxt.text = ConvertSpeedMStoKMH(rigid_body.velocity.magnitude).ToString("F0");// rigid_body.velocity.magnitude shows speed meters per second (m/s)
        }

        updateWheels();
        RearMudGuardSuspension();
        CalcGear();

        if (Input.GetKey(KeyCode.R) && bike_fallen == true)
            Reset();

        ////////////////////////////////////////////////////////////////
        // Store current step count for any tests:
        ////////////////////////////////////////////////////////////////

        step_count_prev = step_count;
    }

    private void motoControlRHB(BikeInputRHB bike_input_rhb, int step_count, 
        out BikeCoords bike_coords, out BikePose bike_pose, ref SteerCalc steer_calc_data)
    {


        ////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////

        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////
        // Bike position and velocity:
        ////////////////////////////////////////////////////////////////

        Vector3 pos_bike = thisTransform.position;
        Vector3 dt_pos_bike = (pos_bike - pos_bike_prev) / dt_step;
        Vector3 dir_unit_bike = GetBikeDirectionVector();  

        pos_bike_prev = pos_bike;
        dt_pos_bike_magn = dt_pos_bike.magnitude;

        ////////////////////////////////////////////////////////////////
        // "Control" angle and angular vel (see Simple Motorcycle Physics, p. 9):
        ////////////////////////////////////////////////////////////////

        float angle_ctrl = Vector3.Dot(dir_unit_bike, Vector3.Cross(thisTransform.up, new Vector3(0, 1, 0)));
        float dt_angle_ctrl = (angle_ctrl - angle_ctrl_prev) / dt_step;

        angle_ctrl_prev = angle_ctrl;
        dt_angle_ctrl_prev = dt_angle_ctrl;

        ////////////////////////////////////////////////////////////////
        // Bike roll angle (rad):
        ////////////////////////////////////////////////////////////////

        if (transform.eulerAngles.z > 180)
            angle_roll_bike = FACT_DEG_2_RAD * transform.eulerAngles.z - 2 * (float)Math.PI;
        else
            angle_roll_bike = FACT_DEG_2_RAD * transform.eulerAngles.z;

        ////////////////////////////////////////////////////////////////
        // Angular roll speed:
        ////////////////////////////////////////////////////////////////

        float dt_angle_roll_bike = (angle_roll_bike - angle_roll_bike_prev) / dt_step;

        angle_roll_bike_prev = angle_roll_bike;

        ////////////////////////////////////////////////////////////////
        // Track the multiple updates to bike_input_rhb.steer that happen in this function:
        ////////////////////////////////////////////////////////////////

        float[] steer_update = new float[N_STEER_UPDATES + 1];

        ////////////////////////////////////////////////////////////////
        // Initialize bike_input_rhb.steer updates:
        ////////////////////////////////////////////////////////////////

        steer_update[0] = bike_input_rhb.steer;

        if (USE_STEER_UPDATE_FULL)
        {
            ////////////////////////////////////////////////////////////////
            // bike_input_rhb.steer UPDATE 1 (update angle_ctrl, dt_angle_ctrl):
            ////////////////////////////////////////////////////////////////

            float ratio_speed = 0f;

            // Low-speed case (steer update):
            if (dt_pos_bike_magn < SPEED_REF_LOW)
            {
                ratio_speed = dt_pos_bike_magn / SPEED_REF_LOW;

                angle_ctrl *= (2.0f - ratio_speed);
                dt_angle_ctrl *= ratio_speed * ratio_speed;

                bike_input_rhb.acceleration += 3.0f * Mathf.Abs(angle_ctrl) * (1.0f - ratio_speed);

                steer_update[1] = bike_input_rhb.steer * ratio_speed * ratio_speed;
                bike_input_rhb.steer = steer_update[1];
            }
            // High-speed case (NO steer update):
            else if (dt_pos_bike_magn > SPEED_REF_HIGH)
            {
                ratio_speed = dt_pos_bike_magn / SPEED_REF_HIGH;

                // Adjust roll angular speed for return to upright:
                if (dt_angle_ctrl * angle_ctrl < 0f)
                    dt_angle_ctrl *= FACTOR_DT_ANGLE_CTRL_RETURN * ratio_speed;

                steer_update[1] = bike_input_rhb.steer;
            }
            // Other case (NO steer update):
            else
                steer_update[1] = bike_input_rhb.steer;

            ////////////////////////////////////////////////////////////////
            // bike_input_rhb.steer UPDATE 2 (update steer input with angle_ctrl squared):   
            ////////////////////////////////////////////////////////////////         

            steer_update[2] = bike_input_rhb.steer * (1 - FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER * (float)Math.Pow(angle_ctrl, 2));

            bike_input_rhb.steer = steer_update[2];
        }
        else
        {
            steer_update[1] = bike_input_rhb.steer;
            steer_update[2] = bike_input_rhb.steer;
        }

        ////////////////////////////////////////////////////////////////
        // bike_input_rhb.steer UPDATE 3 (steer terms weighted sum: input, angle_ctrl, dt_angle_ctrl):
        ////////////////////////////////////////////////////////////////

        // Bike speed factor - update:
        // factor_steer_bike_speed = 7.6e-4f; // minimum from data
        factor_steer_bike_speed = 1f / (1f + (float)Math.Pow(dt_pos_bike_magn, 2));

        // Steer input factor - moved here from steerHelper() (TODO: keep or discard):
        // factor_steer_input = Mathf.Clamp(FACTOR_STEER_INPUT - 0.9f / FACT_DEG_2_RAD * Mathf.Abs(angle_roll_bike), 10f, FACTOR_STEER_INPUT);
        factor_steer_input = FACTOR_STEER_INPUT;

        // Control angle input factor - moved here from steerHelper() (TODO: keep or d iscard):
        // if (Math.Abs(angle_roll_bike) > ANGLE_ROLL_LOW_DEG)
        //    factor_steer_angle_ctrl += 3.0f; // 2.0f;

        float steer_term_input         = factor_steer_input * bike_input_rhb.steer;
        float steer_term_angle_ctrl    = factor_steer_angle_ctrl * angle_ctrl;
        float steer_term_dt_angle_ctrl = FACTOR_STEER_DT_ANGLE_CTRL * dt_angle_ctrl;

        steer_update[3] = factor_steer_bike_speed *
            (steer_term_input + steer_term_angle_ctrl + steer_term_dt_angle_ctrl);

        bike_input_rhb.steer = steer_update[3];

        ////////////////////////////////////////////////////////////////
        // bike_input_rhb.steer UPDATE 4 (clamp with input_steer_prev):
        ////////////////////////////////////////////////////////////////

        float inc_steer = FACTOR_INC_STEER * dt_step;
        
        // steer_update[4] = Mathf.Clamp(bike_input_rhb.steer, input_steer_prev - inc_steer, input_steer_prev + inc_steer);
        steer_update[4] = steer_update[3];

        bike_input_rhb.steer = steer_update[4];

        ////////////////////////////////////////////////////////////////
        // Save steering value for next step:
        ////////////////////////////////////////////////////////////////

        input_steer_prev = bike_input_rhb.steer;

        ////////////////////////////////////////////////////////////////
        // Update wheels steering angle (wheel colliders) - CRITICAL:
        ////////////////////////////////////////////////////////////////

        if (dt_pos_bike_magn > SPEED_TRANSITION_ANGLE_STEER_BEHAV)
            wheel_coll_fwd.steerAngle = bike_input_rhb.steer * ANGLE_STEER_FRONT_WHEEL_MAX_DEG; // Mathf.Clamp(bike_input_rhb.steer, -1f, 1f) * ANGLE_STEER_FRONT_WHEEL_MAX_DEG;
        else
            wheel_coll_fwd.steerAngle = Mathf.Clamp(bike_input_rhb.steer, -dt_pos_bike_magn, dt_pos_bike_magn); // TODO: how come there is no scaling

        ////////////////////////////////////////////////////////////////
        // Apply input for torque & force control - CTRITICAL:
        ////////////////////////////////////////////////////////////////

        float scale_factor_accel = 0f;

        if (USE_RHB_THROTTLE) {
            wheel_coll_back.motorTorque = torque_motor * bike_input_rhb.throttle;

            if (dt_pos_bike_magn < SPEED_REF_HIGH)
                scale_factor_accel = 1f;
            else
                scale_factor_accel = 0.5f;

            rigid_body.AddForce(scale_factor_accel * FACTOR_ACCEL * bike_input_rhb.throttle * transform.forward);
        }
        else {
            wheel_coll_back.motorTorque = torque_motor * bike_input_rhb.acceleration;

            if (dt_pos_bike_magn < SPEED_REF_HIGH)
                rigid_body.AddForce(FACTOR_ACCEL * bike_input_rhb.acceleration * transform.forward);
        }

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
            steer_calc_data.steer_update[i] = steer_update[i];

        steer_calc_data.factor_steer_bike_speed  = factor_steer_bike_speed;
        steer_calc_data.steer_term_input         = steer_term_input;
        steer_calc_data.steer_term_angle_ctrl    = steer_term_angle_ctrl;
        steer_calc_data.steer_term_dt_angle_ctrl = steer_term_dt_angle_ctrl;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

        if (step_count % (DT_DISP_DATA_MSEC / ReHandyBotController.DT_STEP_APP_MSEC) == 0 && step_count > step_count_prev &&
            ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            ExternalConsoleLogger.Log("    factor_speed_steer [" + String.Format("{0:#0.000}", factor_steer_bike_speed) + "]\n");
            // ExternalConsoleLogger.Log("    steer_term_input   [" + String.Format("{0:#0.000}", steer_term_input) + "]");
            // ExternalConsoleLogger.Log("    steer_term_angle   [" + String.Format("{0:#0.000}", steer_term_angle_ctrl) + "]");
            // ExternalConsoleLogger.Log("    steer_term_dt_angle[" + String.Format("{0:#0.000}", steer_term_dt_angle_ctrl) + "]");

            ExternalConsoleLogger.Log("    factor_steer_input         [" + factor_steer_input         + "]");
            ExternalConsoleLogger.Log("    factor_steer_angle_ctrl    [" + factor_steer_angle_ctrl    + "]");
            ExternalConsoleLogger.Log("    FACTOR_STEER_DT_ANGLE_CTRL [" + FACTOR_STEER_DT_ANGLE_CTRL + "]");

            ExternalConsoleLogger.Log("\n");

            /*
            for (int j = 0; j <= N_STEER_UPDATES; j++)
                ExternalConsoleLogger.Log("    steer_update[" + j + "] = " + String.Format("{0:#0.000}", steer_update[j]));

            ExternalConsoleLogger.Log("\n");
            */
        }

        /*
        if (step_count % DECIM_DATA_DISP_BIKE_CTRL == 0 && step_count > step_count_prev &&
            ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            try
            {
                ExternalConsoleLogger.Log("    " +
                    "wheel_coll_fwd.motorTorque [" + wheel_coll_fwd.motorTorque + "]");
                ExternalConsoleLogger.Log("    " +
                    "wheel_coll_back.motorTorque [" + wheel_coll_back.motorTorque + "]");
            }
            catch (Exception exc)
            {
                ExternalConsoleLogger.Log("   -------------------------------------------------------------");
                ExternalConsoleLogger.Log("   motoControlRHB(): EXCEPTION - failed to access wheel collider\n");
                ExternalConsoleLogger.Log("      Exception message: [" + exc.Message + "]");
                ExternalConsoleLogger.Log("      Stack trace:       [" + exc.StackTrace + "] \n");
            }

            ExternalConsoleLogger.Log(" ");
        }
        */
    }

    void Awake()
    {
        instance = this;
    }

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

    private void uprightForce(bool input_force_trq_on)
    {
        rigid_body.angularDrag -= 100f * Time.deltaTime;
        rigid_body.angularDrag = Mathf.Clamp(rigid_body.angularDrag, 0.1f, 100f);

        if (dt_pos_bike_magn < 1.0f && !input_force_trq_on) // Input.GetKey(KeyCode.W) (11.06.2025)
        {
            // Removed 11.06.2025: 
            // var rot = Quaternion.FromToRotation(transform.up, Vector3.up);
            // rigid_body.AddTorque(new Vector3(rot.x, rot.y, rot.z)* 10 , ForceMode.Acceleration);

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
        handles.transform.localRotation = Quaternion.Euler(0, Mathf.Lerp(handles.transform.localRotation.y, wheel_coll_fwd.steerAngle, Time.deltaTime * 10), 0);
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

    public float ConvertSpeedMStoKMH(float speed)
    {
        return Mathf.Clamp(speed * 3.6f, 0, FACTOR_ACCEL);
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
