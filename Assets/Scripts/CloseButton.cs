using UnityEngine;
using UnityEngine.UI;

public class CloseButton : MonoBehaviour
{
    public void QuitApplication()
    {
        Debug.Log("Quitting Application");
        Application.Quit();
    }
}