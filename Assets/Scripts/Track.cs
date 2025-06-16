using UnityEngine;
using System.Collections.Generic;

public class Track : MonoBehaviour
{
    public static Track Instance;

    [Space]
    [SerializeField] private Transform waypointsParent;
    [SerializeField] private bool splineCurve = false;

    List<Transform> waypoints = new();
    List<float> distances = new();

    private void Awake()
    {
        Instance = this;
    }

    private void Start()
    {
        // Set up the list of way points to use later for the rest of the calculations
        SetupWaypointList();

        // Calculate the total distance covered by the track
        SetupTrackDistances();
    }

    private void SetupWaypointList()
    {
        waypoints = new();

        foreach (Transform t in waypointsParent)
        {
            waypoints.Add(t);
        }
    }

    private void SetupTrackDistances()
    {
        float totalTrackDistance = 0f;
        distances = new();
        distances.Add(0);
        
        for (int i = 1; i < waypoints.Count; i++)
        {
            totalTrackDistance += Vector3.Distance(waypoints[i - 1].position, waypoints[i].position);
            distances.Add(totalTrackDistance);
        }

        totalTrackDistance += Vector3.Distance(waypoints[^1].position, waypoints[0].position);
        distances.Add(totalTrackDistance);
    }

    #region Helper functions
    /// <summary>
    /// Returns the world position on the centerline at the specified distance along the track.
    /// Useful for placing objects or evaluating movement at a specific point on the track.
    /// Automatically wraps around if the distance exceeds the total track length.
    /// </summary>
    /// <param name="dist">Distance along the track in world units.</param>
    /// <returns>World position on the centerline at the given distance.</returns>
    public Vector3 GetPositionAtDistance(float dist)
    {
        dist = Mathf.Repeat(dist, distances[^1]);

        int i = 1;
        while (i < distances.Count && distances[i] < dist)
        {
            i++;
        }

        int p1 = i - 1;
        int p2 = i % waypoints.Count;

        float t = Mathf.InverseLerp(distances[p1], distances[i], dist);
        return Vector3.Lerp(waypoints[p1].position, waypoints[p2].position, t);
    }

    /// <summary>
    /// Returns the distance along the centerline that is closest to the given world position.
    /// Useful for determining how far along the track a position is.
    /// </summary>
    /// <param name="pos">World position to evaluate.</param>
    /// <returns>Distance (in world units) along the track to the closest point on the centerline.</returns>
    public float GetDistanceAtPosition(Vector3 pos)
    {
        float closestDist = float.MaxValue;
        float projectedDistance = 0f;

        for (int i = 0; i < waypoints.Count; i++)
        {
            int j = (i + 1) % waypoints.Count;

            Vector3 a = waypoints[i].position;
            Vector3 b = waypoints[j].position;

            Vector3 ab = b - a;
            Vector3 ap = pos - a;

            float t = Mathf.Clamp01(Vector3.Dot(ap, ab) / ab.sqrMagnitude);
            Vector3 projection = a + ab * t;

            float sqrDist = (pos - projection).sqrMagnitude;
            if (sqrDist < closestDist)
            {
                closestDist = sqrDist;
                float segmentStart = distances[i];
                float segmentEnd = distances[i + 1];
                projectedDistance = Mathf.Lerp(segmentStart, segmentEnd, t);
            }
        }

        return projectedDistance;
    }

    /// <summary>
    /// Returns the closest world position on the centerline to the given position.
    /// Useful for snapping objects to the center of the track.
    /// </summary>
    /// <param name="position">World position to evaluate.</param>
    /// <returns>Closest point on the centerline in world coordinates.</returns>
    public Vector3 GetClosestPointOnCenterLine(Vector3 position)
    {
        float closestSqrDist = float.MaxValue;
        Vector3 closestPoint = Vector3.zero;

        for (int i = 0; i < waypoints.Count; i++)
        {
            int j = (i + 1) % waypoints.Count;

            Vector3 a = waypoints[i].position;
            Vector3 b = waypoints[j].position;

            Vector3 ab = b - a;
            Vector3 ap = position - a;

            float t = Mathf.Clamp01(Vector3.Dot(ap, ab) / ab.sqrMagnitude);
            Vector3 projection = a + ab * t;

            float sqrDist = (position - projection).sqrMagnitude;
            if (sqrDist < closestSqrDist)
            {
                closestSqrDist = sqrDist;
                closestPoint = projection;
            }
        }

        return closestPoint;
    }

    /// <summary>
    /// Returns the rotation aligned with the track direction near the given world position.
    /// Useful for spawning objects or aligning them to follow the centerline.
    /// </summary>
    /// <param name="pos">World position to evaluate.</param>
    /// <returns>Quaternion representing track-aligned rotation at the closest waypoint segment.</returns>
    public Quaternion GetTrackRotationAtPosition(Vector3 pos)
    {
        float closestSqrDist = float.MaxValue;
        int closestIndex = 0;

        for (int i = 0; i < waypoints.Count; i++)
        {
            float sqrDist = (waypoints[i].position - pos).sqrMagnitude;
            if (sqrDist < closestSqrDist)
            {
                closestSqrDist = sqrDist;
                closestIndex = i;
            }
        }

        int nextIndex = (closestIndex + 1) % waypoints.Count;
        Vector3 dir = (waypoints[nextIndex].position - waypoints[closestIndex].position).normalized;

        return Quaternion.LookRotation(dir, Vector3.up);
    }
    #endregion

    #region Debugging functions
    private readonly int curveResolution = 20;
    private readonly float angleThreshold = 5f;

    private void OnDrawGizmos()
    {
        SetupWaypointList();

        if (splineCurve)
        {
            if (waypoints == null || waypoints.Count < 4)
            return;

            Gizmos.color = Color.cyan;

            for (int i = 0; i < waypoints.Count; i++)
            {
                Vector3 p0 = waypoints[(i - 1 + waypoints.Count) % waypoints.Count].position;
                Vector3 p1 = waypoints[i % waypoints.Count].position;
                Vector3 p2 = waypoints[(i + 1) % waypoints.Count].position;
                Vector3 p3 = waypoints[(i + 2) % waypoints.Count].position;

                float angle = Vector3.Angle(p2 - p1, p1 - p0);
                if (Mathf.Abs(angle - 180f) < angleThreshold)
                {
                    Gizmos.DrawLine(p1, p2);
                    continue;
                }

                Vector3 prevPos = p1;
                for (int j = 1; j <= curveResolution; j++)
                {
                    float t = j / (float)curveResolution;
                    Vector3 pos = GetCatmullRomPosition(t, p0, p1, p2, p3);
                    Gizmos.DrawLine(prevPos, pos);
                    prevPos = pos;
                }
            }
        }
        else
        {
            SetupWaypointList();

            if (waypoints == null || waypoints.Count < 2)
                return;

            Gizmos.color = Color.white;
            for (int i =  1; i < waypoints.Count; i++)
            {
                Gizmos.DrawLine(waypoints[i - 1].position, waypoints[i].position);
            }

            Gizmos.DrawLine(waypoints[^1].position, waypoints[0].position);
        }
    }
    
    private Vector3 GetCatmullRomPosition(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float t2 = t * t;
        float t3 = t2 * t;

        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t3
        );
    }
    #endregion
}