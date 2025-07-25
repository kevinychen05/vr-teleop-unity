using UnityEngine;

public class ThumbPinchGrab : MonoBehaviour
{
    public OVRHand hand;
    public Transform thumbTipTransform;
    public bool useSmoothing = false;
    public float followSpeed = 10f;

    private bool isTouchingCube = false;
    private bool isHolding = false;
    private GameObject touchedCube = null;

    private Vector3 positionOffset;
    private Quaternion rotationOffset;

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Grabbable"))
        {
            isTouchingCube = true;
            touchedCube = other.gameObject;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Grabbable"))
        {
            isTouchingCube = false;

            if (!isHolding)
            {
                touchedCube = null;
            }
        }
    }

    void Update()
    {
        bool isPinching = hand.GetFingerIsPinching(OVRHand.HandFinger.Index);

        // Start holding
        if (isTouchingCube && !isHolding && isPinching)
        {
            isHolding = true;

            // Calculate initial offset at moment of grabbing
            positionOffset = touchedCube.transform.position - thumbTipTransform.position;
            rotationOffset = Quaternion.Inverse(thumbTipTransform.rotation) * touchedCube.transform.rotation;
        }

        // While holding, follow with offset
        if (isHolding && touchedCube != null)
        {
            Vector3 targetPosition = thumbTipTransform.position + positionOffset;
            Quaternion targetRotation = thumbTipTransform.rotation * rotationOffset;

            if (useSmoothing)
            {
                touchedCube.transform.position = Vector3.Lerp(
                    touchedCube.transform.position,
                    targetPosition,
                    Time.deltaTime * followSpeed
                );

                touchedCube.transform.rotation = Quaternion.Lerp(
                    touchedCube.transform.rotation,
                    targetRotation,
                    Time.deltaTime * followSpeed
                );
            }
            else
            {
                touchedCube.transform.position = targetPosition;
                touchedCube.transform.rotation = targetRotation;
            }

            // Release when unpinched
            if (!isPinching)
            {
                isHolding = false;
                touchedCube = null;
            }
        }
    }
}
