using UnityEngine;

public class CurvatureCalculator : MonoBehaviour
{
    public Transform[] waypoints; // Assign the waypoints in the inspector
    public float[] curvatures; // Array to store curvature values

    private void Start()
    {
        if (waypoints == null || waypoints.Length < 3)
        {
            Debug.LogError("Waypoints array is empty or has less than 3 elements.");
            return;
        }

        for (int i = 0; i < waypoints.Length; i++)
        {
            if (waypoints[i] == null)
            {
                Debug.LogError("Waypoint " + i + " is null.");
                return;
            }
        }

        CalculateCurvature();
    }

    private void CalculateCurvature()
    {
        curvatures = new float[waypoints.Length - 2];

        for (int i = 1; i < waypoints.Length - 1; i++)
        {
            Vector3 p0 = waypoints[i - 1].position;
            Vector3 p1 = waypoints[i].position;
            Vector3 p2 = waypoints[i + 1].position;

            // Calculate the curvature using the formula:
            // κ = |(p1 - p0) x (p2 - p1)| / (|(p1 - p0)| * |(p2 - p1)|)
            // Notes: This formula calculates the curvature at each point along the curve defined by the waypoints.
            // The curvature is calculated as the magnitude of the cross product of the vectors (p1 - p0) and (p2 - p1),
            // divided by the product of the magnitudes of the vectors (p1 - p0) and (p2 - p1).

            Vector3 v1 = p1 - p0;
            Vector3 v2 = p2 - p1;

            float numerator = Vector3.Cross(v1, v2).magnitude;
            float denominator = v1.magnitude * v2.magnitude;

            curvatures[i - 1] = numerator / denominator;
        }
    }

    private void OnDrawGizmos()
    {
        for (int i = 0; i < waypoints.Length; i++)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawSphere(waypoints[i].position, 0.1f);
        }

        for (int i = 0; i < curvatures.Length; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(waypoints[i + 1].position, waypoints[i + 1].position + new Vector3(0f, curvatures[i] * 10f, 0f));
        }
    }
}
