using System.Collections;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;
using Unity.Robotics.UrdfImporter;

/// <summary>
/// Closed-loop client for the Drogon C++ backend (LQR + inverse dynamics).
///
/// Flow per RequestPlanQ() call:
///   1. POST /arm/set_reference  →  backend plans a minimum-jerk reference
///                                  trajectory of length T at step dt and
///                                  stores it for the LQR controller.
///   2. Loop, every dt seconds, until t >= T:
///        a. Read current joint state (q, dq) from Unity ArticulationBody.
///        b. POST /arm/step  with {q, dq, t, dt, mode, weights, N, u_max}.
///        c. Backend runs finite-horizon LQR + inverse dynamics, returns
///           q_cmd (next desired joint position).
///        d. Apply q_cmd to ArticulationBody drive targets.
///        e. Yield WaitForSeconds(dt) and increment t.
///
/// The C++ backend prints the per-episode metrics summary
/// (eq_rms_global, max_abs_eq, eq_final_norm, u_energy_total, success)
/// once t reaches T.
/// </summary>
public class Ur5eTrajectoryClientQ : MonoBehaviour
{
    [Header("Backend")]
    public string serverIP = "127.0.0.1";
    public int port = 8848;

    [Header("Trajectory params")]
    public float T = 1.5f;
    public float dt = 0.02f;

    [Header("LQR weights")]
    public float wq   = 30.0f;
    public float wdq  =  2.0f;
    public float wu   =  0.1f;
    public float wqN  = 30.0f;
    public float wdqN =  2.0f;

    [Header("Controller mode and horizon")]
    [Tooltip("\"lqr\", \"mpc_lite\" or \"pd\"")]
    public string mode = "lqr";
    public int horizonN = 20;
    public float u_max = 8.0f;

    [Header("Playback speed")]
    [Tooltip("1 = real time, 2 = 2x slower, 0.5 = 2x faster")]
    public float playbackScale = 1.0f;

    [Header("Applier (must have q1..q6 assigned)")]
    public ApplyJointAngles6 applier;

    Coroutine playRoutine;

    // ---------- JSON DTOs ----------
    [System.Serializable]
    class SetReferenceRequest
    {
        public float[] q_target;
        public float[] q_start;
        public float T;
        public float dt;
    }

    [System.Serializable]
    class WeightsObj
    {
        public float wq;
        public float wdq;
        public float wu;
        public float wqN;
        public float wdqN;
    }

    [System.Serializable]
    class StepRequest
    {
        public float[] q;
        public float[] dq;
        public float t;
        public float dt;
        public string mode;
        public int N;
        public float u_max;
        public WeightsObj weights;
    }

    [System.Serializable]
    class StepResponse
    {
        public bool ok;
        public float t;
        public float dt;
        public string mode;
        public float[] q_cmd;
        public float[] dq_cmd;
        public float[] ddq_cmd;
        public float[] tau_cmd;
        public float[] q_ref;       // posición de referencia minimum-jerk en t
        public float[] dq_ref;
        public float[] ddq_ref;
    }

    string Url(string path) => $"http://{serverIP}:{port}{path}";

    // ====================================================================
    // Public API — same name/signature as before for scene compatibility.
    // Inspector buttons or Ur5eControlPanel call this.
    // ====================================================================
    public void RequestPlanQ(float[] qTargetRad)
    {
        if (applier == null)
        {
            Debug.LogError("[ClosedLoopClient] applier is NULL (assign ApplyJointAngles6 in Inspector).");
            return;
        }
        if (qTargetRad == null || qTargetRad.Length < 6)
        {
            Debug.LogError("[ClosedLoopClient] qTargetRad must be length 6.");
            return;
        }

        StopPlayback();
        playRoutine = StartCoroutine(RunClosedLoop(qTargetRad));
    }

    public void StopPlayback()
    {
        if (playRoutine != null)
        {
            StopCoroutine(playRoutine);
            playRoutine = null;
        }
    }

    // ====================================================================
    // Joint state readers
    // ====================================================================
    float ReadJointRad(UrdfJointRevolute j)
    {
        if (j == null) return 0f;
        var ab = j.GetComponentInChildren<ArticulationBody>(true);
        if (ab == null) ab = j.GetComponent<ArticulationBody>();
        if (ab != null) return ab.jointPosition[0];

        float z = j.transform.localEulerAngles.z;
        if (z > 180f) z -= 360f;
        return z * Mathf.Deg2Rad;
    }

    float ReadJointVelRad(UrdfJointRevolute j)
    {
        if (j == null) return 0f;
        var ab = j.GetComponentInChildren<ArticulationBody>(true);
        if (ab == null) ab = j.GetComponent<ArticulationBody>();
        if (ab != null) return ab.jointVelocity[0];
        return 0f;
    }

    float[] GetUnityCurrentQRad()
    {
        if (applier != null && applier.q1 != null && applier.q2 != null && applier.q3 != null &&
            applier.q4 != null && applier.q5 != null && applier.q6 != null)
        {
            return new float[6]
            {
                ReadJointRad(applier.q1),
                ReadJointRad(applier.q2),
                ReadJointRad(applier.q3),
                ReadJointRad(applier.q4),
                ReadJointRad(applier.q5),
                ReadJointRad(applier.q6),
            };
        }

        if (applier != null && applier.LastQRad != null && applier.LastQRad.Length >= 6)
        {
            return new float[6]
            {
                applier.LastQRad[0], applier.LastQRad[1], applier.LastQRad[2],
                applier.LastQRad[3], applier.LastQRad[4], applier.LastQRad[5]
            };
        }

        return new float[6];
    }

    float[] GetUnityCurrentDqRad()
    {
        if (applier != null && applier.q1 != null && applier.q2 != null && applier.q3 != null &&
            applier.q4 != null && applier.q5 != null && applier.q6 != null)
        {
            return new float[6]
            {
                ReadJointVelRad(applier.q1),
                ReadJointVelRad(applier.q2),
                ReadJointVelRad(applier.q3),
                ReadJointVelRad(applier.q4),
                ReadJointVelRad(applier.q5),
                ReadJointVelRad(applier.q6),
            };
        }
        return new float[6];
    }

    // ====================================================================
    // Closed loop coroutine
    // ====================================================================
    IEnumerator RunClosedLoop(float[] qTargetRad)
    {
        // ---------- Phase 1: POST /arm/set_reference ----------
        float[] qStartRad = GetUnityCurrentQRad();

        var setRefBody = new SetReferenceRequest
        {
            q_target = qTargetRad,
            q_start = qStartRad,
            T = T,
            dt = dt
        };

        string setRefJson = JsonConvert.SerializeObject(setRefBody);

        using (var req = new UnityWebRequest(Url("/arm/set_reference"), "POST"))
        {
            req.uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(setRefJson));
            req.downloadHandler = new DownloadHandlerBuffer();
            req.SetRequestHeader("Content-Type", "application/json");
            req.SetRequestHeader("Accept", "application/json");

            yield return req.SendWebRequest();

            if (req.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError("[ClosedLoopClient] /arm/set_reference HTTP error: " + req.error);
                Debug.LogError("[ClosedLoopClient] response: " + req.downloadHandler.text);
                playRoutine = null;
                yield break;
            }
        }

        // ---------- Phase 2: loop /arm/step until t >= T ----------
        var weights = new WeightsObj
        {
            wq = wq, wdq = wdq, wu = wu, wqN = wqN, wdqN = wdqN
        };

        int nSteps = Mathf.CeilToInt(T / Mathf.Max(1e-4f, dt));
        float t = 0f;

        for (int k = 0; k <= nSteps; k++)
        {
            float[] qNow = GetUnityCurrentQRad();
            float[] dqNow = GetUnityCurrentDqRad();

            var stepBody = new StepRequest
            {
                q = qNow,
                dq = dqNow,
                t = t,
                dt = dt,
                mode = mode,
                N = horizonN,
                u_max = u_max,
                weights = weights,
            };

            string stepJson = JsonConvert.SerializeObject(stepBody);

            using (var req = new UnityWebRequest(Url("/arm/step"), "POST"))
            {
                req.uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(stepJson));
                req.downloadHandler = new DownloadHandlerBuffer();
                req.SetRequestHeader("Content-Type", "application/json");
                req.SetRequestHeader("Accept", "application/json");

                yield return req.SendWebRequest();

                if (req.result != UnityWebRequest.Result.Success)
                {
                    Debug.LogError("[ClosedLoopClient] /arm/step HTTP error: " + req.error);
                    Debug.LogError("[ClosedLoopClient] response: " + req.downloadHandler.text);
                    playRoutine = null;
                    yield break;
                }

                StepResponse resp;
                try
                {
                    resp = JsonConvert.DeserializeObject<StepResponse>(req.downloadHandler.text);
                }
                catch (System.Exception e)
                {
                    Debug.LogError("[ClosedLoopClient] /arm/step JSON parse: " + e.Message);
                    playRoutine = null;
                    yield break;
                }

                // Aplicamos q_ref (la trayectoria minimum-jerk planificada en el
                // backend). El campo q_cmd también está disponible (es la
                // integración de la dinámica del backend partiendo de q_real),
                // pero produce incrementos demasiado pequeños para que el
                // ArticulationDrive de Unity acelere a tiempo. Con q_ref Unity
                // sigue el plan suavemente y el backend mide el error real
                // (eq_rms_global, max_abs_eq, eq_final_norm) entre lo que pasa
                // en Unity y lo que pedía el plan.
                if (resp != null && resp.q_ref != null && resp.q_ref.Length >= 6)
                {
                    applier.Apply(resp.q_ref);
                }
                else if (resp != null && resp.q_cmd != null && resp.q_cmd.Length >= 6)
                {
                    applier.Apply(resp.q_cmd);
                }
            }

            t += dt;
            yield return new WaitForSeconds(dt * Mathf.Max(0.0001f, playbackScale));
        }

        playRoutine = null;
    }
}
