using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class ApplyJointAngles6 : MonoBehaviour
{
    // References to the six revolute joints of the robot (must be assigned in Unity Inspector)
    public UrdfJointRevolute q1, q2, q3, q4, q5, q6;

    // Stores the last applied joint configuration in radians
    public float[] LastQRad { get; private set; } = new float[6];

    // Applies a 6-DOF joint vector (in radians) to the robot joints
    public void Apply(float[] q)
    {
        // Validate input vector
        if (q == null || q.Length < 6) return;

        // Store last commanded joint pose
        for (int i = 0; i < 6; i++) LastQRad[i] = q[i];

        // Check that all joints are correctly assigned
        if (q1 == null || q2 == null || q3 == null || q4 == null || q5 == null || q6 == null)
        {
            Debug.LogError("[ApplyJointAngles6] Assign q1..q6 (UrdfJointRevolute).");
            return;
        }

        // Apply joint angles converting from radians to degrees
        q1.UpdateJointState(q[0] * Mathf.Rad2Deg);
        q2.UpdateJointState(q[1] * Mathf.Rad2Deg);
        q3.UpdateJointState(q[2] * Mathf.Rad2Deg);
        q4.UpdateJointState(q[3] * Mathf.Rad2Deg);
        q5.UpdateJointState(q[4] * Mathf.Rad2Deg);
        q6.UpdateJointState(q[5] * Mathf.Rad2Deg);
    }
}

