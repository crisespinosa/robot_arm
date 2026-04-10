using System.Collections;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;
using Unity.Robotics.UrdfImporter;

/// <summary>
/// Closed-loop PPO controller for the UR5e robot arm.
///
/// Architecture:
///   1. Calls /rl/reset on the Drogon backend to start an episode
///   2. Each FixedUpdate: sends obs to PPO server (/ppo/predict) to get action,
///      then sends action to backend (/rl/step) to get next state
///   3. Applies resulting joint angles to Unity ArticulationBody
///
/// Setup:
///   - Drogon backend running on port 8848
///   - PPO inference server running on port 8849
///   - Assign ApplyJointAngles6 component in Inspector
/// </summary>
public class Ur5ePPOController : MonoBehaviour
{
    [Header("Backend (Drogon C++)")]
    public string backendIP = "127.0.0.1";
    public int backendPort = 8848;

    [Header("PPO Inference Server (Python)")]
    public string ppoIP = "127.0.0.1";
    public int ppoPort = 8849;

    [Header("Trajectory Parameters")]
    public float T = 1.5f;
    public float dt = 0.02f;
    public string mode = "mpc_lite";
    public int N = 20;

    [Header("Target Configuration (radians)")]
    [Tooltip("Set target joint angles in radians, then call StartEpisode()")]
    public float[] qTarget = new float[6];

    [Header("Components")]
    public ApplyJointAngles6 applier;

    [Header("Experiment Logger (optional)")]
    public ExperimentLogger logger;

    [Header("Status")]
    [SerializeField] private bool episodeActive = false;
    [SerializeField] private int stepCount = 0;
    [SerializeField] private float currentReward = 0f;
    [SerializeField] private float totalReward = 0f;
    [SerializeField] private bool lastSuccess = false;
    [SerializeField] private float lastEqRms = 0f;

    // ── Public getters for UI panel ──
    public bool  GetEpisodeActive() => episodeActive;
    public int   GetStepCount()     => stepCount;
    public float GetCurrentReward() => currentReward;
    public float GetTotalReward()   => totalReward;
    public bool  GetLastSuccess()   => lastSuccess;
    public float GetLastEqRms()     => lastEqRms;

    // Internal state
    private float[] currentObs;
    private bool waitingForResponse = false;
    private Coroutine episodeCoroutine;

    // JSON classes
    [System.Serializable]
    private class ResetRequest
    {
        public float[] q_start;
        public float[] q_target;
        public float T;
        public float dt;
        public string mode;
        public int N;
    }

    [System.Serializable]
    private class StepRequest
    {
        public float[] action;
    }

    [System.Serializable]
    private class PredictRequest
    {
        public float[] obs;
        public bool deterministic = true;
    }

    [System.Serializable]
    private class PredictResponse
    {
        public bool ok;
        public float[] action;
    }

    [System.Serializable]
    private class ResetResponse
    {
        public bool ok;
        public float[] obs;
        public int obs_dim;
        public int act_dim;
    }

    [System.Serializable]
    private class StepResponse
    {
        public bool ok;
        public float[] obs;
        public float reward;
        public bool done;
        public bool truncated;
        public float t;
        public int step_count;
        public float[] q_cmd;
        public StepInfo info;
    }

    [System.Serializable]
    private class Q6
    {
        public float q0, q1, q2, q3, q4, q5;
        public float[] ToArray() => new float[] { q0, q1, q2, q3, q4, q5 };
    }

    [System.Serializable]
    private class StepInfo
    {
        public float eq_rms;
        public float edq_rms;
        public float u_energy;
        public float du_energy;
        public bool success;
        public bool time_done;
    }

    // ============================================================
    // Public API
    // ============================================================

    /// <summary>
    /// Start a new RL episode with the current qTarget.
    /// Call this from UI buttons, other scripts, or the Inspector context menu.
    /// </summary>
    [ContextMenu("Start Episode")]
    public void StartEpisode()
    {
        if (episodeActive)
        {
            // Debug.LogWarning("[PPOController] Episode already active. Stopping first.");
            StopEpisode();
        }

        episodeCoroutine = StartCoroutine(RunEpisode());
    }

    /// <summary>
    /// Start episode with specific target angles.
    /// </summary>
    public void StartEpisode(float[] target)
    {
        if (target != null && target.Length >= 6)
        {
            for (int i = 0; i < 6; i++)
                qTarget[i] = target[i];
        }
        StartEpisode();
    }

    /// <summary>
    /// Stop the current episode.
    /// </summary>
    [ContextMenu("Stop Episode")]
    public void StopEpisode()
    {
        if (episodeCoroutine != null)
        {
            StopCoroutine(episodeCoroutine);
            episodeCoroutine = null;
        }
        episodeActive = false;
        waitingForResponse = false;
        // Debug.Log("[PPOController] Episode stopped.");
    }

    // Keyboard controls: P = start episode, O = stop episode
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.P) && !episodeActive)
        {
            StartEpisode();
        }
        if (Input.GetKeyDown(KeyCode.O) && episodeActive)
        {
            StopEpisode();
        }
    }

    // ============================================================
    // Episode loop
    // ============================================================

    private IEnumerator RunEpisode()
    {
        // Read current joint angles as start position
        float[] qStart = GetCurrentQRad();

        Debug.Log($"[PPOController] Starting episode: q_start=[{FormatArray(qStart)}] q_target=[{FormatArray(qTarget)}]");

        // Step 1: Reset the backend
        ResetResponse resetResp = null;
        yield return StartCoroutine(PostReset(qStart, qTarget, (resp) => resetResp = resp));

        if (resetResp == null || !resetResp.ok)
        {
            Debug.LogError("[PPOController] Backend reset failed.");
            yield break;
        }

        currentObs = resetResp.obs;
        episodeActive = true;
        stepCount = 0;
        totalReward = 0f;
        lastSuccess = false;

        Debug.Log($"[PPOController] Episode started. obs_dim={resetResp.obs_dim}");

        // Step 2: Control loop
        while (episodeActive)
        {
            // 2a: Get PPO action from observation
            PredictResponse predictResp = null;
            yield return StartCoroutine(PostPredict(currentObs, (resp) => predictResp = resp));

            if (predictResp == null || predictResp.action == null)
            {
                Debug.LogError("[PPOController] PPO predict failed. Stopping episode.");
                episodeActive = false;
                yield break;
            }

            // 2b: Send action to backend
            StepResponse stepResp = null;
            yield return StartCoroutine(PostStep(predictResp.action, (resp) => stepResp = resp));

            if (stepResp == null || !stepResp.ok)
            {
                Debug.LogError("[PPOController] Backend step failed. Stopping episode.");
                episodeActive = false;
                yield break;
            }

            // 2c: Apply joint angles to Unity robot
            if (stepResp.q_cmd != null && stepResp.q_cmd.Length >= 6)
            {
                applier.Apply(stepResp.q_cmd);
            }

            // 2d: Update state
            currentObs = stepResp.obs;
            currentReward = stepResp.reward;
            totalReward += stepResp.reward;
            stepCount = stepResp.step_count;

            if (stepResp.info != null)
            {
                lastEqRms = stepResp.info.eq_rms;
                lastSuccess = stepResp.info.success;
            }

            // 2e: Check if done
            if (stepResp.done)
            {
                episodeActive = false;
                string result = lastSuccess ? "SUCCESS" : "TIMEOUT";
                Debug.Log($"[PPOController] Episode done: {result}, steps={stepCount}, " +
                          $"reward={totalReward:F2}, eq_rms={lastEqRms:F4}");

                // Notificar al logger
                if (logger != null)
                {
                    logger.OnTrajectoryFinished(qTarget, qStart, T, dt);
                }

                yield break;
            }

            // Wait for next physics step
            yield return new WaitForFixedUpdate();
        }
    }

    // ============================================================
    // HTTP communication
    // ============================================================

    private IEnumerator PostReset(float[] qStart, float[] qTgt,
                                   System.Action<ResetResponse> callback)
    {
        var body = new ResetRequest
        {
            q_start = qStart,
            q_target = qTgt,
            T = this.T,
            dt = this.dt,
            mode = this.mode,
            N = this.N
        };

        string url = $"http://{backendIP}:{backendPort}/rl/reset";
        string json = JsonConvert.SerializeObject(body);

        using var req = new UnityWebRequest(url, "POST");
        req.uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(json));
        req.downloadHandler = new DownloadHandlerBuffer();
        req.SetRequestHeader("Content-Type", "application/json");
        req.timeout = 10;

        yield return req.SendWebRequest();

        if (req.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError($"[PPOController] Reset failed: {req.error}");
            callback(null);
            yield break;
        }

        try
        {
            var resp = JsonConvert.DeserializeObject<ResetResponse>(req.downloadHandler.text);
            callback(resp);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[PPOController] Reset JSON parse error: {e.Message}");
            callback(null);
        }
    }

    private IEnumerator PostPredict(float[] obs, System.Action<PredictResponse> callback)
    {
        var body = new PredictRequest { obs = obs, deterministic = true };
        string url = $"http://{ppoIP}:{ppoPort}/ppo/predict";
        string json = JsonConvert.SerializeObject(body);

        using var req = new UnityWebRequest(url, "POST");
        req.uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(json));
        req.downloadHandler = new DownloadHandlerBuffer();
        req.SetRequestHeader("Content-Type", "application/json");
        req.timeout = 5;

        yield return req.SendWebRequest();

        if (req.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError($"[PPOController] PPO predict failed: {req.error}");
            callback(null);
            yield break;
        }

        try
        {
            var resp = JsonConvert.DeserializeObject<PredictResponse>(req.downloadHandler.text);
            callback(resp);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[PPOController] PPO JSON parse error: {e.Message}");
            callback(null);
        }
    }

    private IEnumerator PostStep(float[] action, System.Action<StepResponse> callback)
    {
        var body = new StepRequest { action = action };
        string url = $"http://{backendIP}:{backendPort}/rl/step";
        string json = JsonConvert.SerializeObject(body);

        using var req = new UnityWebRequest(url, "POST");
        req.uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(json));
        req.downloadHandler = new DownloadHandlerBuffer();
        req.SetRequestHeader("Content-Type", "application/json");
        req.timeout = 5;

        yield return req.SendWebRequest();

        if (req.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError($"[PPOController] Step failed: {req.error}");
            callback(null);
            yield break;
        }

        try
        {
            var resp = JsonConvert.DeserializeObject<StepResponse>(req.downloadHandler.text);
            callback(resp);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[PPOController] Step JSON parse error: {e.Message}");
            callback(null);
        }
    }

    // ============================================================
    // Helpers
    // ============================================================

    private float[] GetCurrentQRad()
    {
        if (applier == null) return new float[6];

        // Try reading from ArticulationBody via the applier's joint references
        if (applier.q1 != null)
        {
            return new float[]
            {
                ReadJointRad(applier.q1),
                ReadJointRad(applier.q2),
                ReadJointRad(applier.q3),
                ReadJointRad(applier.q4),
                ReadJointRad(applier.q5),
                ReadJointRad(applier.q6),
            };
        }

        // Fallback to last applied
        if (applier.LastQRad != null && applier.LastQRad.Length >= 6)
            return (float[])applier.LastQRad.Clone();

        return new float[6];
    }

    private float ReadJointRad(UrdfJointRevolute j)
    {
        if (j == null) return 0f;
        var ab = j.GetComponent<ArticulationBody>();
        if (ab != null) return ab.jointPosition[0];
        float z = j.transform.localEulerAngles.z;
        if (z > 180f) z -= 360f;
        return z * Mathf.Deg2Rad;
    }

    private string FormatArray(float[] arr)
    {
        if (arr == null) return "null";
        var parts = new string[arr.Length];
        for (int i = 0; i < arr.Length; i++)
            parts[i] = arr[i].ToString("F3");
        return string.Join(", ", parts);
    }
}
