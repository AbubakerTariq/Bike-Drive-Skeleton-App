using System;
using System.Collections;
using UnityEngine;

public class MotorbikeController : MonoBehaviour
{
    ////////////////////////////////////////////////////////////////////////////
    // Real-time bike steering step (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////
    
    public const int DT_STEP_INPUT_STEER_BIKE_MSEC = 50; // sampling rate for RHB steer input commands 
    public const int DT_STEP_CTRL_BIKE_MSEC = 50;

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - PREDEFINED VALUES:
    ////////////////////////////////////////////////////////////////////////////

    private float ANGLE_STEER_FRONT_WHEEL_MAX_DEG = 60f; // 75f; // 45f;
    private float TORQUE_MOTOR_MAX = 600f; // 500f; 
    private float FACTOR_ACCEL = 2000f; // 1000f; // CRITICAL value: increases top speed but can make turning harder

    private float FACTOR_BRAKE = 0f;
    private float FACTOR_BRAKE_FWD = 400f;
    private float FACTOR_BRAKE_BACK = 400f;

    private float RADIUS_WHEEL = 0.7f;

    [HideInInspector] public float FACTOR_ANGLE_STEER = 56.0f; // 25.0f; // based on FACTOR_ANGLE_STEER_MIN, FACTOR_ANGLE_STEER_MAX
    private float FACTOR_DT_ANGLE_STEER = 30.0f;
    private float FACTOR_ANGLE_SQUARED_STEER = 2.3f;
    private float FACTOR_INC_STEER = 20.0f; // 10.0f;

    [HideInInspector] public float SPEED_M_PER_SEC_LOW = 8.0f;
    [HideInInspector] public float SPEED_M_PER_SEC_HIGH = 25.0f;

    private float ANGLE_NONSLIP_MAX_DEG = 50.0f; // 40.0f;  

    ////////////////////////////////////////////////////////////////////////////
    // Bike control parameters - ADJUSTABLE VALUES - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////

    // Parameters modifying input.steer:
    private float STEER_SENSITIVITY = 10.0f; // 45.0f; // 30.0f;

    // Throttle - input geometry settings:
    private float DIST_RADIAL_THROT_FULL_MM = 2.0f; // 3.0f; // 5.0f; // grippers travel distance for full throttle (mm)
    private float INPUT_THROT_MAX = 1.3f; // this is a function of RADIAL stiffness  
    private float INPUT_THROT_THRESH = 0.6f; // minimum torque for mobility, also dependent on RADIAL stiffness  

    // Steering - input settings:
    private float INPUT_STEER_REF_DEG = 25.0f; //  15.0f; // 30.0f; // reference angle for steering response (the lower the angle the larger the response)



    // Steering - scaling RHB input:
    private float SCALE_STEER_MIN = 1.0f;  
    private float SCALE_STEER_MAX = 2.0f; // make this > 1 to reduce the actual range of RHB rotation  

    private float ANG_SCALE_START = 15f * (float)Math.PI / 180f;
    private float ANG_SCALE_END = 30f * (float)Math.PI / 180f;

    ////////////////////////////////////////////////////////////////////////////
    // Bike 'physical' parts:
    ////////////////////////////////////////////////////////////////////////////

    public static MotorbikeController Instance;

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

    // private Transform transform_initial;

    ////////////////////////////////////////////////////////////////////////////
    // Driving conditions:
    ////////////////////////////////////////////////////////////////////////////

    float steer_sensitivity_init;
    float torque_motor_init; 
    float angle_turn;
    [HideInInspector] public float rpm_value; 
    
    [HideInInspector] public Vector3 velocity_rel_collision;

    // Initial conditions:
    public bool bike_fallen = false;
    public int gear_curr = 1;

    ///////////////////////////////////////////////////////////
    // "Stoppie" parameters (TODO: keep or discard):
    ///////////////////////////////////////////////////////////

    // public float STOPPIE_AMOUNT = 0.3f;     
    // [HideInInspector] public bool canArtificialStoppie = false; [Range(0.1f, 1f)]

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
    // Wheel data:
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

    ///////////////////////////////////////////////////////////
    // Motorbike input:
    ///////////////////////////////////////////////////////////

    public struct MotorbikeInput
    {
        public float steer;
        public float acceleration;
        public float brakeForward;
        public float brakeBack;

        public float throttle_rhb; // NEW: 11.06.2025   
    }

    /////////////////////////////////////////////////////////////
    // Previous kinematic states:
    /////////////////////////////////////////////////////////////

    private Vector3 position_prev = new Vector3();

    private float angle_prev = 0;
    private float dt_angle_prev = 0;

    private float vel_magn = 0;
    private float steer_prev = 0f;

    ///////////////////////////////////////////////////////////
    // Auxiliary variables:
    ///////////////////////////////////////////////////////////

    const bool USE_RHB_THROTTLE = true;
    const bool USE_RHB_STEER = true;

    /////////////////////////////////////////////////////////////
    // Console displays:
    /////////////////////////////////////////////////////////////

    private bool DISP_FIXED_UPDATE_ON = false;
    private bool DISP_MOTOR_CONTROL_ON = false;

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

        // Initialize driving conditions:
        steer_sensitivity_init = STEER_SENSITIVITY;
        torque_motor_init = TORQUE_MOTOR_MAX;

        // Make copy of initial transform values for resetting purposes:
        // transform_initial = thisTransform;
    }

    void FixedUpdate()
    {
        int DECIM_INPUT_STEER_BIKE = DT_STEP_INPUT_STEER_BIKE_MSEC / DT_STEP_CTRL_BIKE_MSEC;

        int step_count = ReHandyBotController.instance.step_count;

        angle_turn = transform.eulerAngles.z;
        if (transform.eulerAngles.z > 180)
            angle_turn = transform.eulerAngles.z - 360;        

        ////////////////////////////////////////////////////////////////
        // Check if bike is balanced:
        ////////////////////////////////////////////////////////////////

        uprightCheck();

        ////////////////////////////////////////////////////////////////
        // If bike is balanced, process control inputs and update bike's kinematic state:
        ////////////////////////////////////////////////////////////////

        if (!bike_fallen)
        {
            var input = new MotorbikeInput();
            float steer_rhb_raw = 0f;

            ////////////////////////////////////////////////////////////////
            // Acceleration input - KEYBOARD:
            ////////////////////////////////////////////////////////////////         

            if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow) && !USE_RHB_THROTTLE)
                input.acceleration = 1f; // this input gets modified at low speeds - see motorControl()
            else
                input.acceleration = 0f;

            ////////////////////////////////////////////////////////////////
            // RHB radial input - throttle:
            ////////////////////////////////////////////////////////////////

            float pos_throttle = ReHandyBotController.instance.DistalData.PositionR;
            float pos_throttle_zero = ReHandyBotController.instance.POS_RADIAL_THROT_ZERO;

            if (ReHandyBotController.instance.ExerciseActive)
                input.throttle_rhb = Mathf.Clamp(-1000f / DIST_RADIAL_THROT_FULL_MM * (pos_throttle - pos_throttle_zero),
                    0f, INPUT_THROT_MAX);
            else
                input.throttle_rhb = 0f;

            ////////////////////////////////////////////////////////////////
            // RHB rotational input - convert to (raw) steering input:
            ////////////////////////////////////////////////////////////////

            // RHB input:
            float pos_rot = ReHandyBotController.instance.DistalData.PositionP;
            float pos_rot_abs = (float)Math.Abs(pos_rot);

            // Scale RHB input:
            float scale_steer;

            if (pos_rot_abs < ANG_SCALE_START)
                scale_steer = SCALE_STEER_MIN;
            else if (pos_rot_abs > ANG_SCALE_END)
                scale_steer = SCALE_STEER_MAX;
            else
                scale_steer = (SCALE_STEER_MAX - SCALE_STEER_MIN) *
                    (pos_rot_abs - ANG_SCALE_START) / (ANG_SCALE_END - ANG_SCALE_START) + SCALE_STEER_MIN;

            float pos_steer = scale_steer * pos_rot;

            if (ReHandyBotController.instance.ExerciseActive && step_count % DECIM_INPUT_STEER_BIKE == 0)
            {
                ////////////////////////////////////////////////////////////////////////////////
                // METHOD 1: angle input with proportionality factor - may be clamped: 
                ////////////////////////////////////////////////////////////////////////////////

                float pos_steer_max = INPUT_STEER_REF_DEG * Mathf.PI / 180f;

                steer_rhb_raw = pos_steer / pos_steer_max;
                // steer_rhb_raw = Mathf.Clamp(steer_rhb_raw, -1, 1);
            }
            else
                steer_rhb_raw = 0f;

            ////////////////////////////////////////////////////////////////
            // Steering input selection:
            ////////////////////////////////////////////////////////////////

            if (USE_RHB_STEER)
            {
                // Steering input - RHB:
                input.steer = steer_rhb_raw;
            }
            else
            {
                // Steering input - KEYBOARD:
                if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
                    input.steer = 1;
                else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
                    input.steer = -1;
            }        

            ////////////////////////////////////////////////////////////////
            // 'Upright force' calculations:
            ////////////////////////////////////////////////////////////////

            bool input_force_trq_on;

            if (USE_RHB_THROTTLE)
            {
                if (input.throttle_rhb >= INPUT_THROT_THRESH)
                    input_force_trq_on = true;
                else
                    input_force_trq_on = false;
            }
            else
            {
                if (input.acceleration > 0f)
                    input_force_trq_on = true;
                else
                    input_force_trq_on = false;
            }

            uprightForce(input_force_trq_on);

            ////////////////////////////////////////////////////////////////
            // Console output:
            ////////////////////////////////////////////////////////////////

            if ((step_count % DECIM_DATA_DISP_BIKE_CTRL) == 0 && step_count > step_count_prev
                && ReHandyBotController.instance.ExerciseActive && DISP_FIXED_UPDATE_ON)
            {
                ExternalConsoleLogger.Log("    ====================================================================");
                ExternalConsoleLogger.Log("    FixedUpdate(" + step_count + "):");
                ExternalConsoleLogger.Log("    " + 
                    "Bike steer RHB raw [" + String.Format("{0:#0.00}", steer_rhb_raw) + "] " +
                    "Bike throttle RHB [" + String.Format("{0:#0.000}", input.throttle_rhb) + "] " +
                    "USE_RHB_THROTTLE [" + USE_RHB_THROTTLE +"]  USE_RHB_STEER [" + USE_RHB_STEER + "] " +
                    "input_force_trq_on [" + input_force_trq_on + "]");

                // ExternalConsoleLogger.Log("    steer keyboard[" + String.Format("{0:#0.00}", input.steer) + "]");
                // ExternalConsoleLogger.Log("    acceleration [" + input.acceleration + "]");

                ExternalConsoleLogger.Log("\n");
            }

            ////////////////////////////////////////////////////////////////
            // Motion commands:
            ////////////////////////////////////////////////////////////////

            motoControlRHB(input, step_count, USE_RHB_THROTTLE);

            steerHelper();
            steerHandles();
        }

        updateWheels();
        RearMudGuardSuspension();
        CalcGear();

        // TODO: keep or discard
        // if (canArtificialStoppie) // for natural stoppie increase forward brake to 50000 and Front Wheel collider forward friction to 5. Around those values a natural stoppie can be performed.
        //    CalcStoppie(); // Requires prevent falling

        if (Input.GetKey(KeyCode.R) && bike_fallen == true)
        {
            Reset();
        }

        ////////////////////////////////////////////////////////////////
        // Store current step count for any tests:
        ////////////////////////////////////////////////////////////////

        step_count_prev = step_count;
    }

    void Awake()
    {
        Instance = this;
    }

    private void Reset()
    {
        // Reset bike position to the closest center point to the bike's current position
        Transform t = GetComponent<Transform>();
        t.position = Track.Instance.GetClosestPointOnCenterLine(t.position) + new Vector3(0f, 0.1f, 0f);

        // Reset bike rotation to align with the rotation of the track
        Quaternion rotation = Track.Instance.GetTrackRotationAtPosition(t.position);
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

    private void motoControlRHB(MotorbikeInput input, int step_count, bool USE_RHB_THROTTLE)
    {
        ////////////////////////////////////////////////////////////////
        // Time step:
        ////////////////////////////////////////////////////////////////
        
        float dt_step = Time.fixedDeltaTime;

        ////////////////////////////////////////////////////////////////
        // Track the multiple udtaes to input.steer that happen in this function:
        ////////////////////////////////////////////////////////////////
        
        const int N_STEER_UPDATES = 4;
        float[] steer_update = new float[N_STEER_UPDATES + 1];
        steer_update[0] = input.steer;

        ////////////////////////////////////////////////////////////////
        // Position and velocity:
        ////////////////////////////////////////////////////////////////

        var position = thisTransform.position;
        var velocity = (position - position_prev) / dt_step;
        position_prev = position;

        vel_magn = velocity.magnitude;
        var vel_unit_vect = velocity.normalized;

        ////////////////////////////////////////////////////////////////
        // Angle and angular vel:
        ////////////////////////////////////////////////////////////////

        var angle = Vector3.Dot(vel_unit_vect, Vector3.Cross(thisTransform.up, new Vector3(0, 1, 0)));
        var dt_angle = (angle - angle_prev) / dt_step;
        angle_prev = angle;
        dt_angle_prev = dt_angle;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

        if ((step_count % DECIM_DATA_DISP_BIKE_CTRL) == 0 && step_count > step_count_prev &&
           ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            ExternalConsoleLogger.Log("    --------------------------------------------------------------------");
            ExternalConsoleLogger.Log("    motoControlRHB (" + step_count + ") dt_step [" + String.Format("{0:#0.000}", dt_step) + "]:");

            // ExternalConsoleLogger.Log("    " +
            //     "input.steer INTIAL[" + String.Format("{0:#0.000}", input.steer) + "]");

            ExternalConsoleLogger.Log("    " +
                "vel_magn [" + String.Format("{0:#0.000}", vel_magn) + "] " +
                "SPEED_LOW [" + String.Format("{0:#0.000}", SPEED_M_PER_SEC_LOW) + "] ");

            ExternalConsoleLogger.Log("\n");
        }

        ////////////////////////////////////////////////////////////////
        // Update steering & angle values based on speed:
        ////////////////////////////////////////////////////////////////

        float ratio_speed = 0f;

        if (vel_magn < SPEED_M_PER_SEC_LOW)
        {
            ratio_speed = vel_magn / SPEED_M_PER_SEC_LOW;

            // input.steer UPDATE 1 (low-speed case):
            steer_update[1] = input.steer * ratio_speed * ratio_speed; 
            input.steer = steer_update[1]; 
                
            angle *= (2.0f - ratio_speed);
            dt_angle *= ratio_speed * ratio_speed;

            input.acceleration += 3.0f * Mathf.Abs(angle) * (1 - ratio_speed);
        }
        else
        {
            // input.steer UPDATE 1 (other case):
            steer_update[1] = input.steer;
        }        
        
        if (vel_magn > SPEED_M_PER_SEC_HIGH)
        {
            ratio_speed = vel_magn / SPEED_M_PER_SEC_HIGH;

            if (dt_angle * angle < 0f)
                dt_angle *= 1.25f*ratio_speed;
        }

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////

        /*
        if (step_count % DECIM_DATA_DISP_BIKE_CTRL == 0 && step_count > step_count_prev &&
            ReHandyBotController.Instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {
            ExternalConsoleLogger.Log("    " +
                "Bike pos [" + position +
                "]  vel_magn [" + String.Format("{0:#0.000}", vel_magn) +
                "]  SPEED_LOW [" + String.Format("{0:#0.000}", SPEED_LOW) +
                "]  SPEED_HIGH [" + String.Format("{0:#0.000}", SPEED_HIGH) +
                "]  ratio_speed [" + String.Format("{0:#0.000}", ratio_speed) + "]\n");

            ExternalConsoleLogger.Log("    " +
                "input.steer MIDDLE [" + String.Format("{0:#0.000}", input.steer) + "] ");
        }
        */

        ////////////////////////////////////////////////////////////////
        // Further update steering:
        ////////////////////////////////////////////////////////////////

        float inc_steer = FACTOR_INC_STEER * dt_step;

        // input.steer UPDATE 2:
        steer_update[2] = input.steer * (1 - FACTOR_ANGLE_SQUARED_STEER * angle * angle); 
        input.steer = steer_update[2];

        float factor_speed_steer = 1f / (vel_magn * vel_magn + 1f);  
        float steer_term_input = STEER_SENSITIVITY * input.steer;
        float steer_term_angle = FACTOR_ANGLE_STEER * angle;
        float steer_term_dt_angle = FACTOR_DT_ANGLE_STEER * dt_angle;

        // input.steer UPDATE 3:
        steer_update[3] = factor_speed_steer *
            (steer_term_input + steer_term_angle + steer_term_dt_angle);
        input.steer = steer_update[3];

        // input.steer UPDATE 4:
        steer_update[4] = Mathf.Clamp(input.steer, steer_prev - inc_steer, steer_prev + inc_steer);
        input.steer = steer_update[4]; 
        
        ////////////////////////////////////////////////////////////////
        // Save steering value for next step:
        ////////////////////////////////////////////////////////////////

        steer_prev = input.steer;

        ////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////
        
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

        ////////////////////////////////////////////////////////////////
        // Update steering angle (wheel colliders):
        ////////////////////////////////////////////////////////////////

        if (vel_magn > 1.0f)
            wheel_coll_fwd.steerAngle = Mathf.Clamp(input.steer, -1, 1) * ANGLE_STEER_FRONT_WHEEL_MAX_DEG;
        else
            wheel_coll_fwd.steerAngle = Mathf.Clamp(input.steer, -vel_magn, vel_magn);

        ////////////////////////////////////////////////////////////////
        // Update brake torques:
        ////////////////////////////////////////////////////////////////

        wheel_coll_fwd.brakeTorque = FACTOR_BRAKE_FWD * input.brakeForward;
        wheel_coll_back.brakeTorque = FACTOR_BRAKE_BACK * input.brakeBack;

        ////////////////////////////////////////////////////////////////
        // Select input for torque & force control:
        ////////////////////////////////////////////////////////////////

        if (USE_RHB_THROTTLE) {
            wheel_coll_back.motorTorque = TORQUE_MOTOR_MAX * input.throttle_rhb;

            if (vel_magn < SPEED_M_PER_SEC_HIGH)
                rigid_body.AddForce(FACTOR_ACCEL * input.throttle_rhb * transform.forward);
            else
                rigid_body.AddForce(0.5f*FACTOR_ACCEL * input.throttle_rhb * transform.forward);
        }
        else {
            wheel_coll_back.motorTorque = TORQUE_MOTOR_MAX * input.acceleration;

            if (vel_magn < SPEED_M_PER_SEC_HIGH)
                rigid_body.AddForce(FACTOR_ACCEL * input.acceleration * transform.forward );
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
        // Console outputs:
        ////////////////////////////////////////////////////////////////

        if (step_count % DECIM_DATA_DISP_BIKE_CTRL == 0 && step_count > step_count_prev &&
            ReHandyBotController.instance.ExerciseActive && DISP_MOTOR_CONTROL_ON)
        {           
            try
            {
                ExternalConsoleLogger.Log("    " +
                    "wheel_coll_fwd.motorTorque [" + wheel_coll_fwd.motorTorque + "]" );
                ExternalConsoleLogger.Log("    " +
                    "wheel_coll_back.motorTorque [" + wheel_coll_back.motorTorque + "]" );
            }
            catch (Exception exc)
            {
                ExternalConsoleLogger.Log("   -------------------------------------------------------------");
                ExternalConsoleLogger.Log("   motoControlRHB(): EXCEPTION - failed to access wheel collider\n");
                ExternalConsoleLogger.Log("      Exception message: [" + exc.Message + "]");
                ExternalConsoleLogger.Log("      Stack trace:       [" + exc.StackTrace + "] \n");
            }

            ExternalConsoleLogger.Log("\n");
        }
    }
        
    private void uprightForce(bool input_force_trq_on)
    {
        rigid_body.angularDrag -= 100f * Time.deltaTime;
        rigid_body.angularDrag = Mathf.Clamp(rigid_body.angularDrag, 0.1f, 100f);

        if (vel_magn < 1.0f && !input_force_trq_on) // Input.GetKey(KeyCode.W) (11.06.2025)
        {
            // var rot = Quaternion.FromToRotation(transform.up, Vector3.up);
            // rigid_body.AddTorque(new Vector3(rot.x, rot.y, rot.z)* 10 , ForceMode.Acceleration);

            transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, 0);
            rigid_body.constraints = RigidbodyConstraints.FreezeAll;
        }
        else
        {
            rigid_body.constraints = RigidbodyConstraints.None;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        velocity_rel_collision = collision.relativeVelocity;
        if (collision.relativeVelocity.magnitude > 30)
            HardHit = true;
    }

    public void uprightCheck()
    {
        if ((Mathf.Abs(angle_turn) > ANGLE_NONSLIP_MAX_DEG || Input.GetKeyDown(KeyCode.F) || HardHit == true) && bike_fallen == false)
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
        float delta = Time.fixedDeltaTime;

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
            {
                localPos.y = wheel_this.wheelStartPos.y;
            }

            wheel_this.rotation = Mathf.Repeat(wheel_this.rotation + delta * wheel_this.wheelCollider.rpm * 360.0f / 60.0f, 360f);
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

    /*
    private void CalcStoppie()
    {
        var stoppieAngle = transform.eulerAngles.x;
        if (transform.eulerAngles.x > 180)
            stoppieAngle = transform.eulerAngles.x - 360;
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.Space))
            com.z += (vel_magn * Time.deltaTime) / 5;
        else
            com.z -= Time.deltaTime * 100;
        if (com.z < 0 || stoppieAngle > 5 + vel_magn)
        {
            com.z = 0;
            if (stoppieAngle > 50)
                com.z -= stoppieAngle / 10;
        }

        else if (com.z > STOPPIE_AMOUNT)
            com.z = STOPPIE_AMOUNT;
    }
    */

    void steerHelper()
    {
        ////////////////////////////////////////////////////////////////////////////////////
        // Adjust steering sensitivity:
        ////////////////////////////////////////////////////////////////////////////////////
        
        STEER_SENSITIVITY = Mathf.Clamp(steer_sensitivity_init - 0.9f * Mathf.Abs(angle_turn), 10f, steer_sensitivity_init);

        ////////////////////////////////////////////////////////////////////////////////////
        // Adjust steering angle factor (TODO: what do the numbers mean?):
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        float FACTOR_ANGLE_STEER_MIN = 48f;
        float FACTOR_ANGLE_STEER_MAX = 65f;

        float ANGLE_TURN_LOW = 42.0f;

        FACTOR_ANGLE_STEER = Mathf.Clamp(FACTOR_ANGLE_STEER, 
            FACTOR_ANGLE_STEER_MIN, FACTOR_ANGLE_STEER_MAX);

        // Removed 07.08.2025: function is unclear; seems to reduce maneuverability
        /*
        if (Input.anyKey)
            FACTOR_ANGLE_STEER -= 1f;
        else
            FACTOR_ANGLE_STEER += 1f;
        */

        if (angle_turn < -ANGLE_TURN_LOW || angle_turn > ANGLE_TURN_LOW)
            FACTOR_ANGLE_STEER += 3f; // 2f;

        ////////////////////////////////////////////////////////////////////////////////////
        // Adjust torque based on key inputs: 
        ////////////////////////////////////////////////////////////////////////////////////

        /*
        if (angle_turn > 10 && Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
        {
            rigid_body.AddTorque(-transform.forward * 0.1f * angle_turn, ForceMode.Acceleration);
        }

        else if (angle_turn > 20 && Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
        {
            rigid_body.AddTorque(-rigid_body.angularVelocity * 2, ForceMode.Acceleration);
        }
        else if (angle_turn < -10 && Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
        {
            rigid_body.AddTorque(transform.forward * 0.1f * -angle_turn, ForceMode.Acceleration);
        }
        else if (angle_turn < -20 && Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
        {
            rigid_body.AddTorque(-rigid_body.angularVelocity * 2, ForceMode.Acceleration);
        }
        */

        ////////////////////////////////////////////////////////////////////////////////////
        // Sets sideways friction with speed gradations:
        ////////////////////////////////////////////////////////////////////////////////////

        if (vel_magn < 10)
            SetWheelFriction(1.5f);
        else if(vel_magn < 20 && vel_magn > 10)
            SetWheelFriction(2);
        else if(vel_magn < 30 && vel_magn > 20)
            SetWheelFriction(2.5f);
        else if(vel_magn < 40 && vel_magn > 20)
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

        gear_curr = Mathf.FloorToInt(vel_magn / FACT_GEAR);

        if (gear_curr != gear_prev)
            StartCoroutine(MotorDisengage());

        rpm_value = vel_magn % FACT_GEAR / FACT_GEAR;
    }

    IEnumerator MotorDisengage()
    {
        TORQUE_MOTOR_MAX = 0;
        yield return new WaitForSeconds(0.1f);
        TORQUE_MOTOR_MAX = torque_motor_init;
    }

    public Vector3 GetBikePosition()
    {
        return transform.position;
    }
    
    public Vector3 GetBikeDirectionVector()
    {
        return transform.forward;
    }

    public Vector3 GetBikeVelocityVector()
    {
        return rigid_body.velocity;
    }
}
