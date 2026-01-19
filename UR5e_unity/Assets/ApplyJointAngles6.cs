using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class ApplyJointAngles6 : MonoBehaviour
{
    public UrdfJointRevolute q1, q2, q3, q4, q5, q6;

    public float[] LastQRad { get; private set; } = new float[6];

    public void Apply(float[] q)
    {
        if (q == null || q.Length < 6) return;

        // store last commanded pose
        for (int i = 0; i < 6; i++) LastQRad[i] = q[i];

        if (q1 == null || q2 == null || q3 == null || q4 == null || q5 == null || q6 == null)
        {
            Debug.LogError("[ApplyJointAngles6] Assign q1..q6 (UrdfJointRevolute).");
            return;
        }

        q1.UpdateJointState(q[0] * Mathf.Rad2Deg);
        q2.UpdateJointState(q[1] * Mathf.Rad2Deg);
        q3.UpdateJointState(q[2] * Mathf.Rad2Deg);
        q4.UpdateJointState(q[3] * Mathf.Rad2Deg);
        q5.UpdateJointState(q[4] * Mathf.Rad2Deg);
        q6.UpdateJointState(q[5] * Mathf.Rad2Deg);
    }
}

