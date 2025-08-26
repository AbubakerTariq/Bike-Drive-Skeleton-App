using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class MotorbikeController : MonoBehaviour
{
    ////////////////////////////////////////////////////////////////////////////
    // Real-time bike steering step (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    public const int DT_STEP_INPUT_STEER_BIKE_MSEC = 25; // sampling rate for RHB steer input commands 

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - PREDEFINED / INITIAL VALUES:
    ////////////////////////////////////////////////////////////////////////////

    const float ANGLE_STEER_FRONT_WHEEL_MAX_DEG = 60f; // 75f; // 45f;
    const float SPEED_TRANSITION_ANGLE_STEER_BEHAV = 1.0f; // transition speed for wheel steering angle behavior

    const float TORQUE_MOTOR_MAX = 600f; // 500f;                                         
    const float FACTOR_ACCEL = 2000f; // 1000f; // CRITICAL value: increases top speed but can make turning harder

    const float FACTOR_BRAKE = 0f;
    const float FACTOR_BRAKE_FWD = 400f;
    const float FACTOR_BRAKE_BACK = 400f;

    const float RADIUS_WHEEL = 0.7f;

    const float FACTOR_STEER_ANGLE_CTRL_REF        = 56.0f; // // based on range of 48 to 65 deg in original code
    const float FACTOR_STEER_DT_ANGLE_CTRL            = 30.0f;
    const float FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER = 2.3f;

    const float FACTOR_INC_STEER = 20.0f; // 10.0f;

    const float SPEED_REF_LOW_M_PER_SEC  =  8.0f; 
    const float SPEED_REF_HIGH_M_PER_SEC = 25.0f;

    // Roll and nonslip limit angles:
    public const float ANGLE_ROLL_LOW_DEG         = 42.0f;
    public const float ANGLE_ROLL_NONSLIP_MAX_DEG = 50.0f;

    // Return to vertical: scaling factor for roll angular speed
    const float FACTOR_DT_ANGLE_CTRL_RETURN = 1.25f;

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - TUNABLE VALUES - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////

    // Parameters modifying bike_input_rhb.steer:
    const float STEER_INPUT_SENSITIVITY = 10.0f; // 45.0f; // 30.0f;

    // Throttle - input geometry settings:
    const float DIST_RADIAL_THROT_FULL_MM = 2.0f; // grippers travel distance for full throttle (mm)

    const float INPUT_THROT_MAX    = 1.3f; // this is a function of RADIAL stiffness  
    const float INPUT_THROT_THRESH = 0.6f; // minimum torque for mobility, also dependent on RADIAL stiffness  

    // Steering - input settings:
    const float POS_ROT_STEER_REF_DEG = 25.0f; //  15.0f; // 30.0f; // reference angle for steering response (the lower the angle the larger the response)

    // Steering - scaling RHB input:
    const float SCALE_STEER_RHB_MIN = 1.0f;
    const float SCALE_STEER_RHB_MAX = 2.0f; // make this > 1 to reduce the actual range of RHB rotation  

    const float ANG_SCALE_POS_ROT_START = 15.0f * (float)Math.PI / 180f;
    const float ANG_SCALE_POS_ROT_END   = 30.0f * (float)Math.PI / 180f;

    ////////////////////////////////////////////////////////////////////////////
    // Object instance:
    ////////////////////////////////////////////////////////////////////////////

    public static MotorbikeController instance;

    ////////////////////////////////////////////////////////////////////////////
    // Driving conditions (with initial values):
    ////////////////////////////////////////////////////////////////////////////

    [HideInInspector] public float factor_steer_angle_ctrl = FACTOR_STEER_ANGLE_CTRL_REF;

    [HideInInspector] public float speed_ref_low  = SPEED_REF_LOW_M_PER_SEC;
    [HideInInspector] public float speed_ref_high = SPEED_REF_HIGH_M_PER_SEC;

    [HideInInspector] public float rpm_value;
    [HideInInspector] public Vector3 velocity_rel_collision;

    private float factor_steer_input = STEER_INPUT_SENSITIVITY;
    private float factor_steer_bike_speed = 0f;

    private float torque_motor = 0f;
    private float angle_roll_deg = 0f;

    public bool bike_fallen = false;
    public int gear_curr = 1;

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

    private const int DECIM_DATA_DISP_BIKE_CTRL = 50;

    private int step_count_prev = 0;

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
        public float angle_roll;
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
    // Public DATA VARIABLES for sharing data among classes:
    /////////////////////////////////////////////////////////////   

    public BikeCoords bike_coords_data = new(); // Motorbike coordinates
    public BikeInputRHB bike_input_rhb_data = new(); // Motorbike input
    public BikePose bike_pose_data = new(); // Motorbike pose
    public TrackCoords track_coords_data = new(); // Track coordinates

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
       
    // Throttle input:
    const bool USE_RHB_THROTTLE = true;

    // Steering mode:
    const int STEER_MODE_RHB        = 1;
    const int STEER_MODE_KEYB       = 2;
    const int STEER_MODE_TRACK_CTRL = 3;

    const int CASE_STEER_MODE = STEER_MODE_TRACK_CTRL;

    /////////////////////////////////////////////////////////////
    // Display settings:
    /////////////////////////////////////////////////////////////

    private bool DISP_FIXED_UPDATE_ON = false;
    private bool DISP_MOTOR_CONTROL_ON = false;

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
        // Bike roll angle (degrees):
        ////////////////////////////////////////////////////////////////

        angle_roll_deg = transform.eulerAngles.z;

        if (transform.eulerAngles.z > 180)
            angle_roll_deg = transform.eulerAngles.z - 360;

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
            // RHB radial input - throttle:
            ////////////////////////////////////////////////////////////////

            float pos_throttle      = ReHandyBotController.instance.distal_data.PositionR;
            float pos_throttle_zero = ReHandyBotController.instance.POS_RADIAL_BASE_THROT;

            if (ReHandyBotController.instance.ExerciseActive)
                bike_input_rhb.throttle = Mathf.Clamp(-1000f / DIST_RADIAL_THROT_FULL_MM * (pos_throttle - pos_throttle_zero),
                    0f, INPUT_THROT_MAX);
            else
                bike_input_rhb.throttle = 0f;

            ////////////////////////////////////////////////////////////////
            // RHB rotational input - convert to RAW steering input:
            ////////////////////////////////////////////////////////////////

            float pos_rot_steer_ref = POS_ROT_STEER_REF_DEG * Mathf.PI / 180f;

            // RHB input:
            float pos_rot = ReHandyBotController.instance.distal_data.PositionP;
            float pos_rot_abs = (float)Math.Abs(pos_rot);

            // Scale RHB input:
            float scale_steer_rhb;

            if (pos_rot_abs < ANG_SCALE_POS_ROT_START)
                scale_steer_rhb = SCALE_STEER_RHB_MIN;
            else if (pos_rot_abs > ANG_SCALE_POS_ROT_END)
                scale_steer_rhb = SCALE_STEER_RHB_MAX;
            else
                scale_steer_rhb = (SCALE_STEER_RHB_MAX - SCALE_STEER_RHB_MIN) *
                    (pos_rot_abs - ANG_SCALE_POS_ROT_START) / (ANG_SCALE_POS_ROT_END - ANG_SCALE_POS_ROT_START) + SCALE_STEER_RHB_MIN;

            ////////////////////////////////////////////////////////////////
            // Select steering mode:
            ////////////////////////////////////////////////////////////////

            // Steering input: RHB / TRACKING CONTROL:
            if (CASE_STEER_MODE == STEER_MODE_RHB || CASE_STEER_MODE == STEER_MODE_TRACK_CTRL)
            {
                if (ReHandyBotController.instance.ExerciseActive)
                {
                    float pos_rot_val;

                    // Select position value
                    if (CASE_STEER_MODE == STEER_MODE_RHB)
                        pos_rot_val = pos_rot;
                    else
                        pos_rot_val = ReHandyBotController.instance.haptic_ctrl_data.pos_rot_ref;

                    // Angle input with proportionality factor:   
                    bike_input_rhb.steer = scale_steer_rhb * pos_rot_val / pos_rot_steer_ref;

                    // Removed 23.08.2025:
                    // bike_input.steer = Mathf.Clamp(bike_input_rhb.steer, -1f, 1f); 
                }
                else
                    bike_input_rhb.steer = 0f;
            }
                
            // Steering input - KEYBOARD:
            else if (CASE_STEER_MODE == STEER_MODE_KEYB)
            {
                if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
                    bike_input_rhb.steer = 1;
                else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
                    bike_input_rhb.steer = -1;
            }

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
            // Display section:
            ////////////////////////////////////////////////////////////////

            /*
            if ((step_count % DECIM_DATA_DISP_BIKE_CTRL) == 0 && step_count > step_count_prev
                && ReHandyBotController.instance.ExerciseActive && DISP_FIXED_UPDATE_ON)
            {
                ExternalConsoleLogger.Log("    ====================================================================");
                ExternalConsoleLogger.Log("    FixedUpdate(" + step_count + "):");
                ExternalConsoleLogger.Log("    " +
                    "Bike throttle RHB [" + String.Format("{0:#0.000}", bike_input_rhb.throttle) + "] " +
                    "USE_RHB_THROTTLE [" + USE_RHB_THROTTLE + "]  USE_RHB_STEER [" + USE_RHB_STEER + "] " +
                    "input_force_trq_on [" + input_force_trq_on + "]");

                ExternalConsoleLogger.Log(" ");
            }
            */

            ////////////////////////////////////////////////////////////////
            // Bike control commands (CRITICAL):
            ////////////////////////////////////////////////////////////////            

            motoControlRHB(bike_input_rhb, step_count, USE_RHB_THROTTLE, out bike_coords_data, out bike_pose_data);

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

    private void motoControlRHB(BikeInputRHB bike_input_rhb, int step_count, bool use_rhb_throttle,
        out BikeCoords bike_coords, out BikePose bike_pose)
    {
        ////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////

        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////
        // Track the multiple updates to bike_input_rhb.steer that happen in this function:
        ////////////////////////////////////////////////////////////////

        const int N_STEER_UPDATES = 4;
        float[] steer_update = new float[N_STEER_UPDATES + 1];

        ////////////////////////////////////////////////////////////////
        // Initialize bike_input_rhb.steer updates:
        ////////////////////////////////////////////////////////////////

        steer_update[0] = bike_input_rhb.steer;

        ////////////////////////////////////////////////////////////////
        // Position and velocity:
        ////////////////////////////////////////////////////////////////

        Vector3 pos_bike = thisTransform.position;
        Vector3 dt_pos_bike = (pos_bike - pos_bike_prev) / dt_step;
        Vector3 dir_unit_bike = GetBikeDirectionVector(); // was dt_pos_bike.normalized (TODO: verify this)

        pos_bike_prev = pos_bike;
        dt_pos_bike_magn = dt_pos_bike.magnitude;

        ////////////////////////////////////////////////////////////////
        // Angle and angular vel:
        ////////////////////////////////////////////////////////////////

        // Control angle (see Simple Motorcycle Physics, p. 9):
        float angle_ctrl = Vector3.Dot(dir_unit_bike, Vector3.Cross(thisTransform.up, new Vector3(0, 1, 0)));
        float dt_angle_ctrl = (angle_ctrl - angle_ctrl_prev) / dt_step;

        angle_ctrl_prev = angle_ctrl;
        dt_angle_ctrl_prev = dt_angle_ctrl;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

        /*
        if ((step_count % DECIM_DATA_DISP_BIKE_CTRL) == 0 && step_count > step_count_prev &&
           ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            ExternalConsoleLogger.Log("    --------------------------------------------------------------------");
            ExternalConsoleLogger.Log("    motoControlRHB (" + step_count + ") dt_step [" + String.Format("{0:#0.000}", dt_step) + "]:");

            // ExternalConsoleLogger.Log("    " +
            //     "bike_input_rhb.steer INTIAL[" + String.Format("{0:#0.000}", bike_input_rhb.steer) + "]");

            ExternalConsoleLogger.Log("    " +
                "vel_magn [" + String.Format("{0:#0.000}", dt_pos_bike_magn) + "] " +
                "SPEED_LOW [" + String.Format("{0:#0.000}", SPEED_LOW_M_PER_SEC) + "] ");

            ExternalConsoleLogger.Log("\n");
        }
        */

        ////////////////////////////////////////////////////////////////
        // Update steering & angle values based on speed:
        ////////////////////////////////////////////////////////////////

        float ratio_speed = 0f;

        ////////////////////////////////////////////////////////////////
        // bike_input_rhb.steer UPDATE 1 (update angle_ctrl, dt_angle_ctrl):
        ////////////////////////////////////////////////////////////////

        // Low-speed case (steer update):
        if (dt_pos_bike_magn < speed_ref_low)
        {
            ratio_speed = dt_pos_bike_magn / speed_ref_low;

            angle_ctrl *= (2.0f - ratio_speed);
            dt_angle_ctrl *= ratio_speed * ratio_speed;

            bike_input_rhb.acceleration += 3.0f * Mathf.Abs(angle_ctrl) * (1.0f - ratio_speed);

            steer_update[1] = bike_input_rhb.steer * ratio_speed * ratio_speed;
            bike_input_rhb.steer = steer_update[1];
        }
        // High-speed case (NO steer update):
        else if (dt_pos_bike_magn > speed_ref_high)
        {
            ratio_speed = dt_pos_bike_magn / speed_ref_high;

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

        steer_update[2] = bike_input_rhb.steer * (1 - FACTOR_STEER_ANGLE_CTRL_SQUARED_STEER * angle_ctrl * angle_ctrl);

        bike_input_rhb.steer = steer_update[2];

        ////////////////////////////////////////////////////////////////
        // bike_input_rhb.steer UPDATE 3 (steer terms weighted sum: input, angle_ctrl, dt_angle_ctrl):
        ////////////////////////////////////////////////////////////////

        // Bike speed factor - update:
        factor_steer_bike_speed = 1.0f / (dt_pos_bike_magn * dt_pos_bike_magn + 1.0f);

        // Steer input factor - update:
        // Moved here from steerHelper() (23.08.2025):
        factor_steer_input = Mathf.Clamp(
            STEER_INPUT_SENSITIVITY - 0.9f * Mathf.Abs(angle_roll_deg),
            10f, STEER_INPUT_SENSITIVITY);

        // Control angle input factor - update:
        // Moved here from steerHelper() (23.08.2025):
        if (Math.Abs(angle_roll_deg) > ANGLE_ROLL_LOW_DEG)
            factor_steer_angle_ctrl += 3.0f; // 2.0f;

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
        
        steer_update[4] = Mathf.Clamp(bike_input_rhb.steer, input_steer_prev - inc_steer, input_steer_prev + inc_steer);

        bike_input_rhb.steer = steer_update[4];

        ////////////////////////////////////////////////////////////////
        // Save steering value for next step:
        ////////////////////////////////////////////////////////////////

        input_steer_prev = bike_input_rhb.steer;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

        /*
        if (step_count % DECIM_DATA_DISP_BIKE_CTRL == 0 && step_count > step_count_prev &&
            ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            ExternalConsoleLogger.Log("    factor_speed_steer [" + String.Format("{0:#0.000}", factor_speed_steer) + "]\n");
            ExternalConsoleLogger.Log("    steer_term_input   [" + String.Format("{0:#0.000}", steer_term_input) + "]");
            ExternalConsoleLogger.Log("    steer_term_angle   [" + String.Format("{0:#0.000}", steer_term_angle) + "]");
            ExternalConsoleLogger.Log("    steer_term_dt_angle[" + String.Format("{0:#0.000}", steer_term_dt_angle) + "]");

            ExternalConsoleLogger.Log("\n");

            for (int j = 0; j <= N_STEER_UPDATES; j++)
            {
                ExternalConsoleLogger.Log("    " +
                    "steer_update[" + j + "] = " + String.Format("{0:#0.000}", steer_update[j]));
            }

            ExternalConsoleLogger.Log("\n");
        }
        */

        ////////////////////////////////////////////////////////////////
        // Update steering angle (wheel colliders):
        ////////////////////////////////////////////////////////////////

        if (dt_pos_bike_magn > SPEED_TRANSITION_ANGLE_STEER_BEHAV)
            wheel_coll_fwd.steerAngle = Mathf.Clamp(bike_input_rhb.steer, -1f, 1f) * ANGLE_STEER_FRONT_WHEEL_MAX_DEG;
        else
            // TODO: how come there is no scaling of dt_pos_bike_magn:
            wheel_coll_fwd.steerAngle = Mathf.Clamp(bike_input_rhb.steer, -dt_pos_bike_magn, dt_pos_bike_magn);

        ////////////////////////////////////////////////////////////////
        // Update brake torques:
        ////////////////////////////////////////////////////////////////

        /*
        wheel_coll_fwd.brakeTorque = FACTOR_BRAKE_FWD * bike_input_rhb.brakeForward;
        wheel_coll_back.brakeTorque = FACTOR_BRAKE_BACK * bike_input_rhb.brakeBack;
        */

        ////////////////////////////////////////////////////////////////
        // Select input for torque & force control:
        ////////////////////////////////////////////////////////////////

        if (use_rhb_throttle) {
            wheel_coll_back.motorTorque = torque_motor * bike_input_rhb.throttle;

            if (dt_pos_bike_magn < speed_ref_high)
                rigid_body.AddForce(FACTOR_ACCEL * bike_input_rhb.throttle * transform.forward);
            else
                rigid_body.AddForce(0.5f * FACTOR_ACCEL * bike_input_rhb.throttle * transform.forward);
        }
        else {
            wheel_coll_back.motorTorque = torque_motor * bike_input_rhb.acceleration;

            if (dt_pos_bike_magn < speed_ref_high)
                rigid_body.AddForce(FACTOR_ACCEL * bike_input_rhb.acceleration * transform.forward);
        }

        ////////////////////////////////////////////////////////////////
        // Update rigid-body Cartesian velocities:
        ////////////////////////////////////////////////////////////////

        if (Input.GetAxis("Vertical") < 0)
            rigid_body.velocity = new Vector3(
                rigid_body.velocity.x * (1.0f - FACTOR_BRAKE / 10f),
                rigid_body.velocity.y,
                rigid_body.velocity.z * (1.0f - FACTOR_BRAKE / 10f));

        ////////////////////////////////////////////////////////////////
        // Generate bike coordinates output struct:
        ////////////////////////////////////////////////////////////////

        bike_coords.pos_bike = pos_bike;
        bike_coords.dt_pos_bike = dt_pos_bike;
        bike_coords.dir_unit_bike = dir_unit_bike;

        ////////////////////////////////////////////////////////////////
        // Generate bike pose output struct:
        ////////////////////////////////////////////////////////////////

        bike_pose.angle_roll = angle_roll_deg * (float)Math.PI / 180f;
        bike_pose.angle_steer_wheel_fwd = wheel_coll_fwd.steerAngle * (float)Math.PI / 180f;
        bike_pose.angle_ctrl = angle_ctrl;
        bike_pose.dt_angle_ctrl = dt_angle_ctrl;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

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
        if ((Mathf.Abs(angle_roll_deg) > ANGLE_ROLL_NONSLIP_MAX_DEG || Input.GetKeyDown(KeyCode.F) || HardHit == true)
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
        ////////////////////////////////////////////////////////////////////////////////////
        // Adjust torque based on key inputs (removed 23.08.2025): 
        ////////////////////////////////////////////////////////////////////////////////////

        /*
        if (!USE_RHB_THROTTLE) {
            if (angle_roll_deg > 10 && Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
                rigid_body.AddTorque(-transform.forward * 0.1f * angle_roll_deg, ForceMode.Acceleration);

            else if (angle_roll_deg > 20 && Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
                rigid_body.AddTorque(-rigid_body.angularVelocity * 2, ForceMode.Acceleration);

            else if (angle_roll_deg < -10 && Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
                rigid_body.AddTorque(transform.forward * 0.1f * -angle_roll_deg, ForceMode.Acceleration);

            else if (angle_roll_deg < -20 && Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
                rigid_body.AddTorque(-rigid_body.angularVelocity * 2, ForceMode.Acceleration);
        }
        */

        ////////////////////////////////////////////////////////////////////////////////////
        // Set sideways friction with speed gradations:
        ////////////////////////////////////////////////////////////////////////////////////

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
