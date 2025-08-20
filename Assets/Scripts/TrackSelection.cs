using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class TrackSelection : MonoBehaviour
{
    public GameObject TestTracks;
    public bool enableTracks;
    // Start is called before the first frame update
    void Start()
    {

    }
    private void Update()
    {
        if (enableTracks)
        {
            TestTracks.SetActive(true);
        }
        else
        {
            TestTracks.SetActive(false);
        }
    }
    public void SelectTrack(int trackIndex)
    {
        if (trackIndex == 0)
        {
            SceneManager.LoadScene("Prototype");
        }
        if (trackIndex == 1)
        {
            SceneManager.LoadScene("Track1");
        }
        if (trackIndex == 2)
        {
            SceneManager.LoadScene("Track2");
        }
        if (trackIndex == 3)
        {
            SceneManager.LoadScene("Track3");
        }
        if (trackIndex == 4)
        {
            SceneManager.LoadScene("Track4");
        }
    }
}
