using UnityEngine;

public class HandPinchDebugger : MonoBehaviour
{
    public OVRHand hand; // Assign this via Inspector

    void Start() {
        Debug.Log("hello at least");
    }

    void Update()
    {
        if (hand == null) return;

        if (hand.GetFingerIsPinching(OVRHand.HandFinger.Index))
        {
            Debug.Log("Pinching with index finger!");
        }
    }

    void OnDisable()
    {
        Debug.Log("HandPinchDebugger was disabled");
    }
}
