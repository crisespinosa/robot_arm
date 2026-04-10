using System.Collections;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

/// <summary>
/// Registra resultados de experimentos PMP en un archivo CSV.
/// Después de cada trayectoria, espera a que el robot se estabilice
/// y guarda: q_target, q_final (real), error por junta y error RMS.
///
/// USO:
///   1. Crear un GameObject vacío "ExperimentLogger"
///   2. Agregar este script
///   3. Asignar en Inspector: client, applier, y las 6 juntas j1..j6
///   4. Ejecutar experimentos normalmente con los sliders
///   5. Al terminar, el CSV estará en Application.dataPath (carpeta Assets/)
/// </summary>
public class ExperimentLogger : MonoBehaviour
{
    [Header("Referencias (asignar en Inspector)")]
    public Ur5eTrajectoryClientQ client;
    public ApplyJointAngles6 applier;

    [Header("URDF Joints (para leer posición REAL del robot)")]
    public UrdfJointRevolute j1, j2, j3, j4, j5, j6;

    [Header("Configuración")]
    [Tooltip("Segundos extra de espera después del playback para que la física se estabilice")]
    public float settleTime = 0.5f;

    [Tooltip("Nombre del archivo CSV de salida")]
    public string csvFileName = "pmp_experiment_results.csv";

    int expCount = 0;
    string csvPath;

    void Awake()
    {
        csvPath = Path.Combine(Application.dataPath, csvFileName);

        if (!File.Exists(csvPath))
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine(
                "exp_id,T,dt," +
                "q_target_1,q_target_2,q_target_3,q_target_4,q_target_5,q_target_6," +
                "q_start_1,q_start_2,q_start_3,q_start_4,q_start_5,q_start_6," +
                "q_final_1,q_final_2,q_final_3,q_final_4,q_final_5,q_final_6," +
                "err_1,err_2,err_3,err_4,err_5,err_6," +
                "err_rms,err_max,success"
            );
            File.WriteAllText(csvPath, sb.ToString());
        }
    }

    /// <summary>
    /// Llamar DESPUÉS de que la trayectoria termine de reproducirse.
    /// Se invoca desde Ur5eTrajectoryClientQ al final de PlayTrajectory.
    /// </summary>
    public void OnTrajectoryFinished(float[] qTarget, float[] qStart, float T, float dt)
    {
        StartCoroutine(LogAfterSettle(qTarget, qStart, T, dt));
    }

    IEnumerator LogAfterSettle(float[] qTarget, float[] qStart, float T, float dt)
    {
        yield return new WaitForSeconds(settleTime);

        float[] qFinal = ReadRealJointPositions();

        float[] errors = new float[6];
        float sumSq = 0f;
        float maxErr = 0f;

        for (int i = 0; i < 6; i++)
        {
            errors[i] = Mathf.Abs(qTarget[i] - qFinal[i]);
            sumSq += errors[i] * errors[i];
            if (errors[i] > maxErr) maxErr = errors[i];
        }

        float errRms = Mathf.Sqrt(sumSq / 6f);
        bool success = errRms < 0.05f; // 0.05 rad ≈ 2.86°

        expCount++;

        StringBuilder sb = new StringBuilder();
        sb.Append(expCount.ToString(CultureInfo.InvariantCulture)).Append(",");
        sb.Append(T.ToString("F3", CultureInfo.InvariantCulture)).Append(",");
        sb.Append(dt.ToString("F4", CultureInfo.InvariantCulture)).Append(",");

        for (int i = 0; i < 6; i++)
            sb.Append(qTarget[i].ToString("F4", CultureInfo.InvariantCulture)).Append(",");

        for (int i = 0; i < 6; i++)
            sb.Append(qStart[i].ToString("F4", CultureInfo.InvariantCulture)).Append(",");

        for (int i = 0; i < 6; i++)
            sb.Append(qFinal[i].ToString("F4", CultureInfo.InvariantCulture)).Append(",");

        for (int i = 0; i < 6; i++)
            sb.Append(errors[i].ToString("F4", CultureInfo.InvariantCulture)).Append(",");

        sb.Append(errRms.ToString("F4", CultureInfo.InvariantCulture)).Append(",");
        sb.Append(maxErr.ToString("F4", CultureInfo.InvariantCulture)).Append(",");
        sb.Append(success ? "1" : "0");

        File.AppendAllText(csvPath, sb.ToString() + System.Environment.NewLine);

        string status = success ? "OK" : "FAIL";
        Debug.Log($"[Exp #{expCount}] errRMS={errRms * Mathf.Rad2Deg:F2}° errMax={maxErr * Mathf.Rad2Deg:F2}° [{status}]");
    }

    float[] ReadRealJointPositions()
    {
        float[] q = new float[6];
        q[0] = ReadOneJoint(j1);
        q[1] = ReadOneJoint(j2);
        q[2] = ReadOneJoint(j3);
        q[3] = ReadOneJoint(j4);
        q[4] = ReadOneJoint(j5);
        q[5] = ReadOneJoint(j6);
        return q;
    }

    float ReadOneJoint(UrdfJointRevolute j)
    {
        if (j == null) return 0f;

        var ab = j.GetComponent<ArticulationBody>()
              ?? j.GetComponentInParent<ArticulationBody>()
              ?? j.GetComponentInChildren<ArticulationBody>();

        if (ab != null && ab.jointPosition.dofCount > 0)
            return ab.jointPosition[0];

        return 0f;
    }
}