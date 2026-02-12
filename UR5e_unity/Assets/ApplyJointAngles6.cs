using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class ApplyJointAngles6 : MonoBehaviour
{
    [Header("Assign the 6 UrdfJointRevolute JOINT objects here (not random links)")]
    public UrdfJointRevolute q1, q2, q3, q4, q5, q6;

    [Header("Drive config (optional but helps)")]
    public bool forceDriveConfig = true;
    public float stiffness = 20000f;
    public float damping = 2000f;
    public float forceLimit = 1000f;

    [Header("Debug")]
    public bool debugMappingOnStart = true;

    public float[] LastQRad { get; private set; } = new float[6];

    ArticulationBody ab1, ab2, ab3, ab4, ab5, ab6;
    bool cached;

    void Awake()
    {
        CacheArticulations();
        if (debugMappingOnStart) DumpMapping();
    }

    void CacheArticulations()
    {
        ab1 = FindChildLinkAB(q1, "q1");
        ab2 = FindChildLinkAB(q2, "q2");
        ab3 = FindChildLinkAB(q3, "q3");
        ab4 = FindChildLinkAB(q4, "q4");
        ab5 = FindChildLinkAB(q5, "q5");
        ab6 = FindChildLinkAB(q6, "q6");
        cached = true;
    }

    ArticulationBody FindChildLinkAB(UrdfJointRevolute joint, string name)
    {
        if (joint == null)
        {
            Debug.LogWarning($"[ApplyJointAngles6] {name} is NULL (not assigned).");
            return null;
        }

        var ab = joint.GetComponentInChildren<ArticulationBody>(true);

        if (ab == null) ab = joint.GetComponent<ArticulationBody>();

        if (ab == null)
            Debug.LogWarning($"[ApplyJointAngles6] {name}: No ArticulationBody found in CHILDREN of joint GO={joint.gameObject.name}");

        return ab;
    }

    void ConfigureDrive(ArticulationBody ab)
    {
        if (ab == null || !forceDriveConfig) return;
        var d = ab.xDrive;
        d.stiffness = stiffness;
        d.damping = damping;
        d.forceLimit = forceLimit;
        ab.xDrive = d;
    }

    void SetTargetRad(ArticulationBody ab, float rad)
    {
        if (ab == null) return;

        ConfigureDrive(ab);

        var d = ab.xDrive;
        d.target = rad * Mathf.Rad2Deg;
        ab.xDrive = d;
    }

    public void Apply(float[] qRad)
    {
        if (qRad == null || qRad.Length < 6) return;
        if (!cached) CacheArticulations();

        for (int i = 0; i < 6; i++) LastQRad[i] = qRad[i];

        SetTargetRad(ab1, qRad[0]);
        SetTargetRad(ab2, qRad[1]);
        SetTargetRad(ab3, qRad[2]);
        SetTargetRad(ab4, qRad[3]);
        SetTargetRad(ab5, qRad[4]);
        SetTargetRad(ab6, qRad[5]);
    }

    void DumpMapping()
    {
        Debug.Log("=== [ApplyJointAngles6] Mapping ===");
        DumpOne("q1", q1, ab1);
        DumpOne("q2", q2, ab2);
        DumpOne("q3", q3, ab3);
        DumpOne("q4", q4, ab4);
        DumpOne("q5", q5, ab5);
        DumpOne("q6", q6, ab6);
        Debug.Log("=== [ApplyJointAngles6] End mapping ===");
    }

    void DumpOne(string name, UrdfJointRevolute j, ArticulationBody ab)
    {
        string jName = (j != null) ? j.gameObject.name : "NULL";
        string abName = (ab != null) ? ab.gameObject.name : "NULL";
        Debug.Log($"[ApplyJointAngles6] {name}: jointGO={jName} -> abGO={abName}");
    }
}
