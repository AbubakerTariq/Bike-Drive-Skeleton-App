using UnityEngine;
using System.Collections.Generic;

public class Track : MonoBehaviour
{
    public static Track Instance;

    [Space]
    [SerializeField] private Transform waypointsParent;

    List<Transform> waypoints = new();
    List<float> distances = new();
    List<Vector3> centerPoints = new();
    private readonly int curveResolution = 20;
    private readonly float angleThreshold = 5f;

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

        // Set up the center line visual
        SetupCenterLineVisual();
    }

    private void SetupWaypointList()
    {
        waypoints = new();
        centerPoints = new();

        foreach (Transform t in waypointsParent)
        {
            waypoints.Add(t);
        }

        if (waypoints == null || waypoints.Count < 4)
            return;

        for (int i = 0; i < waypoints.Count; i++)
        {
            Vector3 p0 = waypoints[(i - 1 + waypoints.Count) % waypoints.Count].position;
            Vector3 p1 = waypoints[i % waypoints.Count].position;
            Vector3 p2 = waypoints[(i + 1) % waypoints.Count].position;
            Vector3 p3 = waypoints[(i + 2) % waypoints.Count].position;

            float angle = Vector3.Angle(p2 - p1, p1 - p0);
            if (Mathf.Abs(angle - 180f) < angleThreshold)
            {
                if (centerPoints.Count == 0 || centerPoints[^1] != p1)
                    centerPoints.Add(p1);
                centerPoints.Add(p2);
                continue;
            }

            Vector3 prevPos = p1;
            if (centerPoints.Count == 0 || centerPoints[^1] != prevPos)
                centerPoints.Add(prevPos);

            for (int j = 1; j <= curveResolution; j++)
            {
                float t = j / (float)curveResolution;
                Vector3 pos = GetCatmullRomPosition(t, p0, p1, p2, p3);
                centerPoints.Add(pos);
            }
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

    private void SetupTrackDistances()
    {
        float totalTrackDistance = 0f;
        distances = new();
        distances.Add(0f);

        for (int i = 1; i < centerPoints.Count; i++)
        {
            totalTrackDistance += Vector3.Distance(centerPoints[i - 1], centerPoints[i]);
            distances.Add(totalTrackDistance);
        }

        // Connect last point to first if the track loops
        totalTrackDistance += Vector3.Distance(centerPoints[^1], centerPoints[0]);
        distances.Add(totalTrackDistance);
    }

    private void SetupCenterLineVisual()
    {
        var lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null || centerPoints == null || centerPoints.Count < 2)
            return;

        int count = centerPoints.Count + 1;
        Vector3[] positions = new Vector3[count];

        for (int i = 0; i < centerPoints.Count; i++)
        {
            positions[i] = centerPoints[i];
        }

        positions[^1] = centerPoints[0]; // Close the loop

        lineRenderer.positionCount = count;
        lineRenderer.SetPositions(positions);
    }

    #region Helper functions
    /// <summary>
    /// Returns the world position on the centerline at the specified distance along the track.
    /// Automatically wraps around if the distance exceeds the total track length.
    /// </summary>
    /// <param name="dist">Distance along the track in world units.</param>
    /// <returns>World position on the centerline at the given distance.</returns>
    public Vector3 GetPositionAtDistance(float dist)
    {
        if (centerPoints == null || centerPoints.Count < 2 || distances == null || distances.Count != centerPoints.Count + 1)
            return Vector3.zero;

        dist = Mathf.Repeat(dist, distances[^1]);

        int i = 1;
        while (i < distances.Count && distances[i] < dist)
        {
            i++;
        }

        int p1 = i - 1;
        int p2 = i % centerPoints.Count;

        float t = Mathf.InverseLerp(distances[p1], distances[i], dist);
        return Vector3.Lerp(centerPoints[p1], centerPoints[p2], t);
    }

    /// <summary>
    /// Returns the distance along the centerline that is closest to the given world position.
    /// </summary>
    /// <param name="pos">World position to evaluate.</param>
    /// <returns>Distance (in world units) along the track to the closest point on the centerline.</returns>
    public float GetDistanceAtPosition(Vector3 pos)
    {
        float closestSqrDist = float.MaxValue;
        float projectedDistance = 0f;

        for (int i = 0; i < centerPoints.Count; i++)
        {
            int j = (i + 1) % centerPoints.Count;

            Vector3 a = centerPoints[i];
            Vector3 b = centerPoints[j];

            Vector3 ab = b - a;
            Vector3 ap = pos - a;

            float t = Mathf.Clamp01(Vector3.Dot(ap, ab) / ab.sqrMagnitude);
            Vector3 projection = a + ab * t;

            float sqrDist = (pos - projection).sqrMagnitude;
            if (sqrDist < closestSqrDist)
            {
                closestSqrDist = sqrDist;
                float segmentStart = distances[i];
                float segmentEnd = distances[i + 1];
                projectedDistance = Mathf.Lerp(segmentStart, segmentEnd, t);
            }
        }

        return projectedDistance;
    }

    /// <summary>
    /// Returns the closest world position on the centerline to the given position.
    /// </summary>
    /// <param name="position">World position to evaluate.</param>
    /// <returns>Closest point on the centerline in world coordinates.</returns>
    public Vector3 GetClosestPointOnCenterLine(Vector3 position)
    {
        float closestSqrDist = float.MaxValue;
        Vector3 closestPoint = Vector3.zero;

        for (int i = 0; i < centerPoints.Count; i++)
        {
            int j = (i + 1) % centerPoints.Count;

            Vector3 a = centerPoints[i];
            Vector3 b = centerPoints[j];

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
    /// </summary>
    /// <param name="pos">World position to evaluate.</param>
    /// <returns>Quaternion representing track-aligned rotation at the closest waypoint segment.</returns>
    public Quaternion GetTrackRotationAtPosition(Vector3 pos)
    {
        float closestSqrDist = float.MaxValue;
        int closestIndex = 0;

        for (int i = 0; i < centerPoints.Count; i++)
        {
            float sqrDist = (centerPoints[i] - pos).sqrMagnitude;
            if (sqrDist < closestSqrDist)
            {
                closestSqrDist = sqrDist;
                closestIndex = i;
            }
        }

        int nextIndex = (closestIndex + 1) % centerPoints.Count;
        Vector3 dir = (centerPoints[nextIndex] - centerPoints[closestIndex]).normalized;

        return Quaternion.LookRotation(dir, Vector3.up);
    }

    /// <summary>
    /// Calculates the curvature of the track at the given world position.
    /// Finds the nearest point on the centerline, samples positions ahead and behind to form a triangle, and uses Heron's formula to compute the radius of curvature.
    /// Returns the inverse of the radius to represent curvature (higher = tighter turn).
    /// </summary>
    /// <param name="position">The world position to evaluate curvature at.</param>
    /// <returns>Curvature value (float), where higher means a sharper turn.</returns>
    public float GetCurvatureAtPosition(Vector3 position)
    {
        float offset = 5f;
        Vector3 centerPos = GetClosestPointOnCenterLine(position);

        float dist = GetDistanceAtPosition(centerPos);

        Vector3 p0 = GetPositionAtDistance(dist - offset);
        Vector3 p1 = GetPositionAtDistance(dist);
        Vector3 p2 = GetPositionAtDistance(dist + offset);

        float a = Vector3.Distance(p0, p1);
        float b = Vector3.Distance(p1, p2);
        float c = Vector3.Distance(p2, p0);

        float s = (a + b + c) * 0.5f;
        float area = Mathf.Sqrt(Mathf.Max(s * (s - a) * (s - b) * (s - c), 0f));

        if (area < 1e-4f) return 0f;

        float radius = (a * b * c) / (4f * area);
        return 1f / radius;
    }

    /// <summary>
    /// Returns the tangent (forward direction) of the centerline at a given distance along the track.
    /// </summary>
    /// <param name="dist">Distance along the track.</param>
    /// <returns>Normalized direction vector representing the tangent at the given distance.</returns>
    public Vector3 GetTangentAtDistance(float dist)
    {
        dist = Mathf.Repeat(dist, distances[^1]);

        int i = 1;
        while (i < distances.Count && distances[i] < dist)
            i++;

        int p1 = i - 1;
        int p2 = i % centerPoints.Count;

        Vector3 dir = (centerPoints[p2] - centerPoints[p1]).normalized;
        return dir;
    }

    /// <summary>
    /// Returns the tangent (forward direction) of the centerline closest to a given world position.
    /// Internally finds the nearest point on the track and returns the direction at that location.
    /// </summary>
    /// <param name="position">World position to evaluate tangent from.</param>
    /// <returns>Normalized direction vector representing the tangent at the closest point on the centerline.</returns>
    public Vector3 GetTangentAtPosition(Vector3 position)
    {
        Vector3 closest = GetClosestPointOnCenterLine(position);
        float dist = GetDistanceAtPosition(closest);
        return GetTangentAtDistance(dist);
    }
    #endregion

    #region Debugging functions
    private void OnDrawGizmos()
    {
        SetupWaypointList();

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
    #endregion
}