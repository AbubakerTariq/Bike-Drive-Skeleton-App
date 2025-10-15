using DG.Tweening;
using Articares.Distal;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using PimDeWitte.UnityMainThreadDispatcher;

public class BikeGameUI : MonoBehaviour
{

    ////////////////////////////////////////////////////////////////////////////
    // Object instances:
    ////////////////////////////////////////////////////////////////////////////

    public static BikeGameUI instance;

    ////////////////////////////////////////////////////////////////////////////
    // Loader variables - moved back to RHBCtrlBike; see notes there (13.10.2025):
    ////////////////////////////////////////////////////////////////////////////

    /*
    [Space] [Header("UI")]
    [SerializeField] private GameObject loader;
    // [SerializeField] private GameObject exerciseGuidelineText;
    [SerializeField] private TMP_Text loaderText;
    */

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // UNITY_GAME: Pre-game actions:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private void Awake()
    {
        instance = this;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Game-specific Unity functions:
    ////////////////////////////////////////////////////////////////////////////

    public void OnConnect_PreUnityGame()
    {
        RHBCtrlBike.instance.SetBrakesRHB(
            RHBCtrlBike.DISENGAGE_BRAKE,
            RHBCtrlBike.DISENGAGE_BRAKE);

        ExternalConsoleLogger.Log("        OnConnect(): SetBrakes(): cmd DISENGAGE \n");
        SetLoaderState(true);

        if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SELECT_BIKE_TYPE)
        {
            SetLoaderText(
               "CLICK on this screen and \n\n" +
               "Select BIKE TYPE: \n\n" +
               "PRO bike: hit [Enter] \n" +
               "Beginner: hit [B]");
        }
    }

    private void OnSelectGameSettings_PreUnityGame()
    {
        if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SET_CTRL_MODE)
        {
            SetLoaderText(
                "Select CONTROL MODE: \n\n" +
                "(1) ASSISTED CONTROL \n" +
                "(2) AUTO STEER / AUTO THROTTLE \n" +
                "(3) AUTO STEER / MANUAL THROTTLE \n" +
                "(4) PURE MANUAL");
        }

        else if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SET_FACT_ASSIST_STEER)
        {
            ////////////////////////////////////////////////////////////////////////////
            // Recommend GAME LEVEL change based on last exercise PERFORMANCE:
            // UNDERSTEER fraction / bike falling / distance traveled:
            ////////////////////////////////////////////////////////////////////////////

            int game_level_next;

            if (RHBCtrlBike.instance.frac_understeer >= 0) // RHBCtrlBike.instance.frac_understeer < 0 means the game hasn't started - no PERFORMANCE metric computed yet
            {
                game_level_next = RHBCtrlBike.instance.PerformGameLevelChange(
                    RHBCtrlBike.instance.game_level_curr,
                    RHBCtrlBike.N_GAME_LEVELS,
                    ref RHBCtrlBike.instance.game_level_change,
                    RHBCtrlBike.instance.frac_understeer,
                    MotorbikeController.instance.step_count_fall,
                    MotorbikeController.instance.dist_traveled, Track.instance.GetTrackLength());

                // TODO: perform this assignment when using automated GAME LEVEL update:
                // GAME_LEVEL_CURR = game_level_next; 
            }

            ////////////////////////////////////////////////////////////////////////////
            // Display section - GAME LEVEL selection message
            ////////////////////////////////////////////////////////////////////////////

            string str_performance;

            if (RHBCtrlBike.instance.frac_understeer >= 0)
                str_performance =
                    "Previous exercise PERFORMANCE: \n\n" +

                    "UNDERSTEER fraction   = [" + String.Format("{0:#0.0}", 100 * RHBCtrlBike.instance.frac_understeer) + " %] \n" +
                    "   max for level UP   = [" + String.Format("{0:#0.0}", 100 * RHBCtrlBike.FRAC_UNDERSTEER_LEVEL_UP_MAX) + " %] \n" +
                    "   min for level DOWN = [" + String.Format("{0:#0.0}", 100 * RHBCtrlBike.FRAC_UNDERSTEER_LEVEL_DOWN_MIN) + " %] \n" +

                    "# falls = [" + MotorbikeController.instance.step_count_fall + "] (limit = " + RHBCtrlBike.N_FALLS_LIM + ")\n\n" +

                    "Dist traveled = [" +
                        String.Format("{0:#0}", MotorbikeController.instance.dist_traveled) + " m] = " +
                        String.Format("{0:#0.0}", 100f * MotorbikeController.instance.dist_traveled / Track.instance.GetTrackLength()) +
                        " % track length (min = " + 100f * RHBCtrlBike.FRAC_LENGTH_TRACK_LEGIT_RACE + "%) \n\n" +

                    "Current GAME LEVEL       = [" + RHBCtrlBike.instance.game_level_curr + "] \n" +
                    "Recommended LEVEL CHANGE = [" + RHBCtrlBike.instance.game_level_change + "] \n\n";
            else
                str_performance = "\n\n";

            SetLoaderText(
                str_performance +
                "\n\n" +
                "Select GAME LEVEL, 1 to 10\n" +
                "(enter [0] for 10) \n");
        }

        else if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SET_FACT_ASSIST_THROTTLE)
        {
            SetLoaderText(
                "Select THROTTLE mode: \n" +
                "(0) MANUAL throttle \n" +
                "(1) AUTO throttle \n");
        }

        else if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_CALIBRATE)
        {
            string str_bike_type;
            string str_game_level;
            string str_fact_assist;
            string str_race_direction;
            string str_game_settings;

            if (RHBCtrlBike.instance.USE_BEGINNER_BIKE_CONSTR)
                str_bike_type = "Bike type: BEGINNER";
            else
                str_bike_type = "Bike type: PRO";

            if (RHBCtrlBike.instance.CASE_CTRL_MODE == RHBCtrlBike.CTRL_ASSISTED)
            {
                str_game_level = "GAME LEVEL = " + RHBCtrlBike.instance.game_level_curr;

                str_fact_assist =
                    "Assist factor STEERING = " + RHBCtrlBike.instance.FACT_ASSIST_STEER + "\n" +
                    "Assist factor THROTTLE = " + RHBCtrlBike.instance.FACT_ASSIST_THROTTLE;

                str_race_direction = "RACE_DIRECTION = [" + RHBCtrlBike.instance.RACE_DIRECTION + "]";

                str_game_settings =
                    str_game_level + "\n" +
                    str_fact_assist + "\n" +
                    str_race_direction;
            }
            else
                str_game_settings = " ";

            SetLoaderText(
                str_bike_type + "\n\n" +
                "CONTROL MODE [" + RHBCtrlBike.instance.CASE_CTRL_MODE + "]\n\n" +
                str_game_settings + "\n\n" +
                "Align grippers horizontally and close the grippers \n\n" +
                "Press Y to CALIBRATE");
        }
    }

    public void InitUnityGame_StartExercise(
        ref bool use_beginner_bike_constr,
        ref int case_ctrl_mode,
        ref int game_level,
        ref float fact_assist_steer,
        ref float fact_assist_throttle,
        ref float frac_pos_rot_input_patient,
        ref float pos_radial_throt_zero,
        ref float k_stiff_radial_throt_manual,
        ref float speed_auto_throttle_max_kph,
        ref int race_direction,
        ref bool upright_constr_on)
    {
        Debug.Log("Start Exercise: ");
        ////////////////////////////////////////////////////////////////////////////
        // Fixed settings for UNITY_GAME:
        ////////////////////////////////////////////////////////////////////////////

        frac_pos_rot_input_patient = 0.4f;
        pos_radial_throt_zero = 0.029f;
        k_stiff_radial_throt_manual = 2500f;
        speed_auto_throttle_max_kph = 150f;

        ////////////////////////////////////////////////////////////////////////////
        // Selectable settings for UNITY_GAME:
        ////////////////////////////////////////////////////////////////////////////

        if (RHBCtrlBike.instance.STATE_PREGAME != RHBCtrlBike.ST_CALIBRATE)
        {
            SelectGameSettings_PreUnityGame(
                ref use_beginner_bike_constr,
                ref case_ctrl_mode,
                ref game_level,
                ref fact_assist_throttle,
                ref race_direction,
                OnSelectGameSettings_PreUnityGame);
            Debug.Log("Game State: " + RHBCtrlBike.instance.STATE_PREGAME + " Calibrated: " + RHBCtrlBike.ST_CALIBRATE);
            // Convert game level to assistance factor (fraction)
            // Modified computation after user feedbacks (29.09.2025)
            fact_assist_steer = RHBCtrlBike.instance.FactorAssistCalc(game_level);
        }

        ////////////////////////////////////////////////////////////////////////////
        // Toggle Exercise state:
        ////////////////////////////////////////////////////////////////////////////

        else if (Input.GetKeyDown(KeyCode.Y))
        {
            RHBCtrlBike.instance.CalibrateRHB(
                RHBCtrlBike.instance.OnCalibrate_CmdStartExercise);

            // Enforce "bike upright" constraint - CRITICAL: 
            MotorbikeController.instance?.uprightConstraintEnforce(ref upright_constr_on); // constraint flag (13.09.2025) 

            // Display section:
            ExternalConsoleLogger.Log("_________________________________________________________________");
            ExternalConsoleLogger.Log("Update(): upright constraint [TRUE] \n");
        }

        ////////////////////////////////////////////////////////////////////////////
        // Toggle Exercise state:
        ////////////////////////////////////////////////////////////////////////////

        if (RHBCtrlBike.instance.isCalibrated && Input.GetKeyDown(KeyCode.Return))
            RHBCtrlBike.instance.ToggleExerciseRHB();
    }

    private void SelectGameSettings_PreUnityGame(
        ref bool use_beginner_bike_constr,
        ref int case_ctrl_mode,
        ref int game_level,
        ref float fact_assist_throttle,
        ref int race_direction,
        UnityAction onComplete = null)
    {
        // TODO: implement selection of race direction:
        race_direction = RHBCtrlBike.DIR_CW;

        ////////////////////////////////////////////////
        // Select BIKE TYPE:
        ////////////////////////////////////////////////

        if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SELECT_BIKE_TYPE)
        {
            if (Input.GetKeyDown(KeyCode.Return))
                use_beginner_bike_constr = false;
            else if (Input.GetKeyDown(KeyCode.B))
                use_beginner_bike_constr = true;
            else
                return;

            RHBCtrlBike.instance.STATE_PREGAME = RHBCtrlBike.ST_SET_CTRL_MODE;

            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select CONTROL MODE:
        ////////////////////////////////////////////////

        if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SET_CTRL_MODE)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
                case_ctrl_mode = RHBCtrlBike.CTRL_ASSISTED;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                case_ctrl_mode = RHBCtrlBike.CTRL_AUTO_STEER_AUTO_THROT;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                case_ctrl_mode = RHBCtrlBike.CTRL_AUTO_STEER_MANUAL_THROT;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                case_ctrl_mode = RHBCtrlBike.CTRL_MANUAL_SIMPLE;
            else
                return;

            if (case_ctrl_mode == RHBCtrlBike.CTRL_ASSISTED)
                RHBCtrlBike.instance.STATE_PREGAME = RHBCtrlBike.ST_SET_FACT_ASSIST_STEER;
            else
                RHBCtrlBike.instance.STATE_PREGAME = RHBCtrlBike.ST_CALIBRATE;

            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select GAME LEVEL manually
        //
        // TODO: this should be replaced by AUTOMATED GAME LEVEL change based on PERFORMANCE
        // See OnSelectGameSettings_PreUnityGame() / if (RHBCtrlBike.instance.STATE_PREGAME == ST_SET_FACT_ASSIST_STEER)
        ////////////////////////////////////////////////

        else if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SET_FACT_ASSIST_STEER)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
                game_level = 1;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                game_level = 2;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                game_level = 3;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                game_level = 4;
            else if (Input.GetKeyDown(KeyCode.Alpha5))
                game_level = 5;
            else if (Input.GetKeyDown(KeyCode.Alpha6))
                game_level = 6;
            else if (Input.GetKeyDown(KeyCode.Alpha7))
                game_level = 7;
            else if (Input.GetKeyDown(KeyCode.Alpha8))
                game_level = 8;
            else if (Input.GetKeyDown(KeyCode.Alpha9))
                game_level = 9;
            else if (Input.GetKeyDown(KeyCode.Alpha0))
                game_level = 10;
            else
                return;

            RHBCtrlBike.instance.STATE_PREGAME = RHBCtrlBike.ST_SET_FACT_ASSIST_THROTTLE;
            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select STEERING assistance factor:
        ////////////////////////////////////////////////

        else if (RHBCtrlBike.instance.STATE_PREGAME == RHBCtrlBike.ST_SET_FACT_ASSIST_THROTTLE)
        {
            if (Input.GetKeyDown(KeyCode.Alpha0))
                fact_assist_throttle = 0f;
            else if (Input.GetKeyDown(KeyCode.Alpha1))
                fact_assist_throttle = 1f;
            else
                return;

            RHBCtrlBike.instance.STATE_PREGAME = RHBCtrlBike.ST_CALIBRATE;
            onComplete.Invoke();
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // Loader functions - TODO: move to BikeGameUI:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    public void SetLoaderState(bool state)
    {
        Debug.Log("Called SetLoader");
        UnityMainThreadDispatcher.Instance().Enqueue(() => RHBCtrlBike.instance.loader.SetActive(state));
    }

    public void SetLoaderText(string text)
    {
        UnityMainThreadDispatcher.Instance().Enqueue(() => RHBCtrlBike.instance.loaderText.text = text);
        UnityMainThreadDispatcher.Instance().Enqueue(() => RHBCtrlBike.instance.loaderText.alignment = TextAlignmentOptions.MidlineLeft);
    }

    /*
    public void SetExerciseGuidelineTextState(bool state)
    {
        exerciseGuidelineText.SetActive(state);
    }
    */
}