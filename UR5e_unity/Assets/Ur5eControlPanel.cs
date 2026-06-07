using UnityEngine;

/// <summary>
/// Runtime UI panel for controlling the UR5e robot arm using the
/// LQR / minimum-jerk pipeline.
///
/// Pipeline driven by this panel:
///     Ur5eTrajectoryClientQ  →  POST /arm/plan_minjerk_q
///                            →  open-loop playback of the planned quintic
///
/// Setup:
///   1. Add this script to any GameObject in the scene.
///   2. Drag the Ur5eTrajectoryClientQ reference into the
///      "Trajectory Client" field in the Inspector.
///   3. Press Play — the panel appears in the top-left corner.
/// </summary>
public class Ur5eControlPanel : MonoBehaviour
{
    [Header("Reference")]
    [Tooltip("Drag your Ur5eTrajectoryClientQ here")]
    public Ur5eTrajectoryClientQ trajectoryClient;

    [Header("Target Configuration (radians)")]
    [Tooltip("Target joint angles in radians")]
    public float[] qTarget = new float[6];

    // ── Joint limits (UR5e, radians) ──
    // Joints 0,3,4,5: ±2π  |  Joints 1,2: -π to π
    private static readonly float[] jointMin = { -6.2832f, -3.1416f, -3.1416f, -6.2832f, -6.2832f, -6.2832f };
    private static readonly float[] jointMax = {  6.2832f,  3.1416f,  3.1416f,  6.2832f,  6.2832f,  6.2832f };

    private static readonly string[] jointNames =
        { "Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3" };

    // ── Layout constants ──
    private const float PANEL_X  = 10f;
    private const float PANEL_Y  = 10f;
    private const float PANEL_W  = 420f;
    private const float PAD      = 6f;
    private const float LABEL_W  = 80f;
    private const float VALUE_W  = 70f;
    private const float SLIDER_W = PANEL_W - LABEL_W - VALUE_W - PAD * 4 - 18f; // 18 = scrollbar

    // ── Internal state ──
    private bool panelVisible = true;
    private Vector2 scrollPos;
    private GUIStyle headerStyle;
    private GUIStyle valueStyle;
    private GUIStyle sectionStyle;
    private bool stylesReady = false;

    // Trajectory param copies (so sliders work smoothly)
    private float trajT;
    private float trajDt;

    // Status tracking (for the status rows)
    private float lastSendTime = -999f;
    private float lastSendT    = 0f;

    void Start()
    {
        if (trajectoryClient == null)
            trajectoryClient = FindObjectOfType<Ur5eTrajectoryClientQ>();

        if (trajectoryClient != null)
        {
            trajT  = trajectoryClient.T;
            trajDt = trajectoryClient.dt;
        }
        else
        {
            trajT  = 1.5f;
            trajDt = 0.02f;
        }

        if (qTarget == null || qTarget.Length < 6) qTarget = new float[6];
    }

    void Update()
    {
        // Keyboard shortcuts
        if (Input.GetKeyDown(KeyCode.Space))
            SendTarget();
        if (Input.GetKeyDown(KeyCode.Escape) && trajectoryClient != null)
            trajectoryClient.StopPlayback();
    }

    private void InitStyles()
    {
        headerStyle = new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            fontSize  = 14,
            normal    = { textColor = Color.white }
        };

        sectionStyle = new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            fontSize  = 11,
            normal    = { textColor = new Color(0.6f, 0.85f, 1f) }
        };

        valueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 11,
            alignment = TextAnchor.MiddleRight,
            normal    = { textColor = new Color(1f, 1f, 0.6f) }
        };

        stylesReady = true;
    }

    void OnGUI()
    {
        if (!stylesReady) InitStyles();

        // Toggle button (always visible)
        if (GUI.Button(new Rect(PANEL_X, PANEL_Y, 30, 24), panelVisible ? "−" : "+"))
            panelVisible = !panelVisible;

        GUI.Label(new Rect(PANEL_X + 34, PANEL_Y + 2, 260, 22),
                  "UR5e LQR Control Panel", headerStyle);

        if (!panelVisible || trajectoryClient == null) return;

        // Panel background
        float contentH = 400f;
        float panelH = Mathf.Min(contentH, Screen.height - PANEL_Y - 20);

        GUI.color = new Color(0, 0, 0, 0.8f);
        GUI.DrawTexture(new Rect(PANEL_X, PANEL_Y + 28, PANEL_W, panelH),
                        Texture2D.whiteTexture);
        GUI.color = Color.white;

        GUILayout.BeginArea(new Rect(PANEL_X + PAD, PANEL_Y + 32,
                                      PANEL_W - PAD * 2, panelH - 8));

        scrollPos = GUILayout.BeginScrollView(scrollPos);

        // ═══════════════════════════════════════
        // TARGET JOINT ANGLES
        // ═══════════════════════════════════════
        GUILayout.Label("Target Joint Angles (rad)", sectionStyle);
        GUILayout.Space(2);

        if (qTarget == null || qTarget.Length < 6) qTarget = new float[6];

        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(jointNames[i], GUILayout.Width(LABEL_W));

            qTarget[i] = GUILayout.HorizontalSlider(
                qTarget[i], jointMin[i], jointMax[i],
                GUILayout.Width(SLIDER_W));

            GUILayout.Label(qTarget[i].ToString("F2"),
                            valueStyle, GUILayout.Width(VALUE_W));
            GUILayout.EndHorizontal();
        }

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Reset to Zero", GUILayout.Height(24)))
        {
            for (int i = 0; i < 6; i++) qTarget[i] = 0f;
        }
        if (GUILayout.Button("Random Target", GUILayout.Height(24)))
        {
            for (int i = 0; i < 6; i++)
                qTarget[i] = Random.Range(jointMin[i] * 0.3f, jointMax[i] * 0.3f);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(8);

        // ═══════════════════════════════════════
        // TRAJECTORY PARAMETERS
        // ═══════════════════════════════════════
        GUILayout.Label("Trajectory Parameters", sectionStyle);
        GUILayout.Space(2);

        // T (duration)
        GUILayout.BeginHorizontal();
        GUILayout.Label("T (s)", GUILayout.Width(LABEL_W));
        trajT = GUILayout.HorizontalSlider(trajT, 0.5f, 5.0f, GUILayout.Width(SLIDER_W));
        GUILayout.Label(trajT.ToString("F2"), valueStyle, GUILayout.Width(VALUE_W));
        GUILayout.EndHorizontal();
        trajectoryClient.T = Mathf.Round(trajT * 20f) / 20f; // snap 0.05

        // dt
        GUILayout.BeginHorizontal();
        GUILayout.Label("dt (s)", GUILayout.Width(LABEL_W));
        trajDt = GUILayout.HorizontalSlider(trajDt, 0.005f, 0.05f, GUILayout.Width(SLIDER_W));
        GUILayout.Label(trajDt.ToString("F3"), valueStyle, GUILayout.Width(VALUE_W));
        GUILayout.EndHorizontal();
        trajectoryClient.dt = Mathf.Round(trajDt * 1000f) / 1000f; // snap 0.001

        // Endpoint (read-only display — useful during the defense)
        GUILayout.BeginHorizontal();
        GUILayout.Label("Endpoint", GUILayout.Width(LABEL_W));
        GUILayout.Label("/arm/set_reference + /arm/step", GUILayout.Width(260));
        GUILayout.EndHorizontal();

        GUILayout.Space(8);

        // ═══════════════════════════════════════
        // TRAJECTORY CONTROL
        // ═══════════════════════════════════════
        GUILayout.Label("Trajectory Control", sectionStyle);
        GUILayout.Space(2);

        GUILayout.BeginHorizontal();
        GUI.backgroundColor = new Color(0.2f, 0.8f, 0.3f);
        if (GUILayout.Button("▶  Send Target", GUILayout.Height(32)))
            SendTarget();

        GUI.backgroundColor = new Color(0.9f, 0.3f, 0.2f);
        if (GUILayout.Button("■  Stop", GUILayout.Height(32)))
            trajectoryClient.StopPlayback();

        GUI.backgroundColor = Color.white;
        GUILayout.EndHorizontal();

        GUILayout.Space(4);
        GUILayout.Label("Keyboard: Space = Send  |  Esc = Stop",
                        new GUIStyle(GUI.skin.label) { fontSize = 10,
                            normal = { textColor = Color.gray } });

        GUILayout.Space(8);

        // ═══════════════════════════════════════
        // LIVE STATUS
        // ═══════════════════════════════════════
        GUILayout.Label("Status", sectionStyle);
        GUILayout.Space(2);

        float elapsed = Time.time - lastSendTime;
        bool  playing = (lastSendTime > 0f) && (elapsed <= lastSendT + 0.25f);

        DrawStatusRow("Playing", playing ? "YES" : "no",
                       playing ? Color.green : Color.gray);
        DrawStatusRow("Last T",
                       lastSendT > 0f ? lastSendT.ToString("F2") + " s" : "-",
                       Color.white);
        DrawStatusRow("Elapsed",
                       lastSendTime > 0f ? elapsed.ToString("F2") + " s" : "-",
                       Color.white);

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void SendTarget()
    {
        if (trajectoryClient == null) return;
        if (qTarget == null || qTarget.Length < 6) return;

        trajectoryClient.StopPlayback();
        trajectoryClient.RequestPlanQ((float[])qTarget.Clone());

        lastSendTime = Time.time;
        lastSendT    = trajectoryClient.T;
    }

    private void DrawStatusRow(string label, string value, Color color)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, GUILayout.Width(LABEL_W));
        var st = new GUIStyle(GUI.skin.label)
        {
            normal = { textColor = color },
            fontStyle = FontStyle.Bold
        };
        GUILayout.Label(value, st);
        GUILayout.EndHorizontal();
    }
}
