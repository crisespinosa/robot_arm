using UnityEngine;

public class KeyboardJogSliders : MonoBehaviour
{
    public Ur5eSliderHandles ui;   // arrastra el objeto que tiene Ur5eSliderHandles
    public float stepDeg = 2f;     // cuánto cambia por tecla (grados)
    public bool enableJog = true;

    float timer = 0f;

    void Update()
    {
        if (!enableJog) return;
        if (ui == null) return;

        timer -= Time.deltaTime;

        bool changed = false;

        // Q/A -> Joint 1
        if (Input.GetKey(KeyCode.Q)) { ui.q1.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.A)) { ui.q1.value -= stepDeg; changed = true; }

        // W/S -> Joint 2
        if (Input.GetKey(KeyCode.W)) { ui.q2.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.S)) { ui.q2.value -= stepDeg; changed = true; }

        // E/D -> Joint 3
        if (Input.GetKey(KeyCode.E)) { ui.q3.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.D)) { ui.q3.value -= stepDeg; changed = true; }

        // R/F -> Joint 4
        if (Input.GetKey(KeyCode.R)) { ui.q4.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.F)) { ui.q4.value -= stepDeg; changed = true; }

        // T/G -> Joint 5
        if (Input.GetKey(KeyCode.T)) { ui.q5.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.G)) { ui.q5.value -= stepDeg; changed = true; }

        // Y/H -> Joint 6
        if (Input.GetKey(KeyCode.Y)) { ui.q6.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.H)) { ui.q6.value -= stepDeg; changed = true; }

        if (changed)
        {
            ui.UpdateLabels();

            // limit how often we send to backend
            if (timer <= 0f)
            {
                ui.SendTarget();       // -> calls client.RequestPlanQ(qRad)
             
            }
        }
    }
}

