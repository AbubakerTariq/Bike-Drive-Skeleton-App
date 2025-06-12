using UnityEngine;
using System.Collections.Generic;

public class Track : MonoBehaviour
{
    [Space]
    [SerializeField] private Transform segmentsParent;
    [SerializeField] private Transform waypointsParent;

    private void OnDrawGizmos()
    {
        Debug.Log("Drawing gizmos");
        List<Transform> waypoints = new();

        foreach (Transform t in waypointsParent)
        {
            waypoints.Add(t);
        }

        Gizmos.color = Color.green;
        for (int i =  0; i < waypoints.Count - 1; i++)
        {
            Gizmos.DrawLine(waypoints[i].position, waypoints[i + 1].position);
        }
    }
}