using UnityEngine;

public class PinchAreaDebugger : MonoBehaviour
{
    void Start() {
        Debug.Log("Start called");
    }

    // void Update() {
    //     Debug.Log("Update running");
    // }

    void OnDisable()
    {
        Debug.Log("PinchAreaDebugger was disabled");
    }

    void OnEnable()
    {
        Debug.Log("PinchAreaDebugger was enabled");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.name == "Cube (1)")
        {
            Debug.Log("Entered pinch area: " + other.name);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.name == "Cube (1)")
        {
            Debug.Log("Exited pinch area: " + other.name);
        }
    }
}
