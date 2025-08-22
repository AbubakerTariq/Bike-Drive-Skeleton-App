using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EngineSoundManager : MonoBehaviour
{
    public float MasterVolume; //in dB
    [SerializeField] AudioSource audioSource, audioSource2, audioSourceWind;
    public AudioClip[] Samples;
    [SerializeField] MotorbikeController motorbikeController;
    [SerializeField] Rigidbody r_body;

    public AnimationCurve EngineRpm, CrossFade, CrossFade2, EngineReleaseRpm;
    [SerializeField] int changed;
    [SerializeField] float prevPitch, prevPitch2, prevVol, prevVol2;
    public bool revLimiter;
    [Range(0, 1)]
    public float revValue;
    public float EngineFlow = 1;
    bool isAccelerated = false;
    void Start()
    {
        //audioSource = GetComponents<AudioSource>()[0];
        //audioSource2 = GetComponents<AudioSource>()[1];
        //audioSourceWind = GetComponents<AudioSource>()[2];
        ChangeGearSound(0);
        //motorbikeController = FindObjectOfType<MotorbikeController>();
    }
    void Update()
    {
        if (revLimiter)
        {
            if (motorbikeController.rpm_value > 0.8f)// && Input.GetKey(KeyCode.W))
            {
                revValue += Time.deltaTime * Random.Range(1, 4);
                revValue %= 1;
                if (revValue > 0.1f && revValue < 0.2f)
                    revValue = 0.85f;
            }
            else
                revValue = motorbikeController.rpm_value;
        }
        else
        {
            revValue = motorbikeController.rpm_value;
        }

        if (changed != motorbikeController.gear_curr)
        {
            changed = motorbikeController.gear_curr;
            if (Input.GetKey(KeyCode.W) || motorbikeController.gear_curr == 0)
                ChangeGearSound(motorbikeController.gear_curr);


            //if (Input.GetKey(KeyCode.W) || motorbikeController.gear_curr == 1)
            //    ChangeGearSound(motorbikeController.gear_curr);
        }

        if (Input.GetKey(KeyCode.W) || ReHandyBotController.instance.distal_data.PositionR < 0.028)
        {
            audioSource.pitch = (EngineRpm.Evaluate(revValue) + 1) - motorbikeController.gear_curr / (Samples.Length - 1);
            audioSource2.pitch = (EngineRpm.Evaluate(revValue) + 1) - motorbikeController.gear_curr / (Samples.Length - 1);
            audioSource.volume = CrossFade.Evaluate(revValue);
            audioSource2.volume = CrossFade2.Evaluate(revValue);
            audioSource.volume = Mathf.Clamp(audioSource.volume, 0f, 0.35f);
            audioSource2.volume = Mathf.Clamp(audioSource2.volume, 0f, 0.35f);
        }
        else
        {
            audioSource.pitch = (EngineReleaseRpm.Evaluate(revValue) + 1) - motorbikeController.gear_curr / (Samples.Length - 1);
            audioSource2.pitch = (EngineReleaseRpm.Evaluate(revValue) + 1) - motorbikeController.gear_curr / (Samples.Length - 1);
        }
        audioSource.pitch = Mathf.Lerp(prevPitch, audioSource.pitch, Time.deltaTime * EngineFlow);
        prevPitch = audioSource.pitch;

        audioSource.outputAudioMixerGroup.audioMixer.SetFloat("VolumeCompensation", MasterVolume - motorbikeController.GetComponent<Rigidbody>().velocity.magnitude / motorbikeController.SPEED_HIGH_M_PER_SEC / 1);
        audioSource.outputAudioMixerGroup.audioMixer.SetFloat("Distortion", (motorbikeController.GetComponent<Rigidbody>().velocity.magnitude / motorbikeController.SPEED_HIGH_M_PER_SEC) / 3 + 0.4f);


        audioSource2.pitch = Mathf.Lerp(prevPitch2, audioSource2.pitch, Time.deltaTime * EngineFlow);
        prevPitch2 = audioSource2.pitch;

        audioSource2.outputAudioMixerGroup.audioMixer.SetFloat("VolumeCompensation", MasterVolume - motorbikeController.GetComponent<Rigidbody>().velocity.magnitude / motorbikeController.SPEED_HIGH_M_PER_SEC / 1);
        audioSource2.outputAudioMixerGroup.audioMixer.SetFloat("Distortion", (motorbikeController.GetComponent<Rigidbody>().velocity.magnitude / motorbikeController.SPEED_HIGH_M_PER_SEC) / 3 + 0.4f);

        //Wind
        audioSourceWind.volume = motorbikeController.GetComponent<Rigidbody>().velocity.magnitude / motorbikeController.SPEED_HIGH_M_PER_SEC + MasterVolume / 10;
        audioSourceWind.volume = Mathf.Clamp(audioSourceWind.volume, 0f, 0.35f);
    }

    void ChangeGearSound(int gear)
    {
        audioSource.Stop();
        audioSource.clip = Samples[gear];
        audioSource.Play();
        audioSource2.Stop();
        audioSource2.clip = Samples[gear + 1];
        audioSource2.Play();
    }
}
