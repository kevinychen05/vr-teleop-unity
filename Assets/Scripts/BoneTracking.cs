using System.Collections.Generic;
using UnityEngine;

public class BoneTracking : MonoBehaviour
{
    // Assign this in the Unity Inspector to the hand object (e.g., LeftHandAnchor or RightHandAnchor)
    public OVRSkeleton handSkeleton;

    // Dictionaries to store bone transforms, positions, and rotations
    public Dictionary<OVRPlugin.BoneId, Transform> boneTransforms = new Dictionary<OVRPlugin.BoneId, Transform>();
    public Dictionary<OVRPlugin.BoneId, Vector3> bonePositions = new Dictionary<OVRPlugin.BoneId, Vector3>();
    public Dictionary<OVRPlugin.BoneId, Quaternion> boneRotations = new Dictionary<OVRPlugin.BoneId, Quaternion>();

    private float logTimer = 0f;  // timer to control 1-second logging

    void Start()
    {
        // Begin tracking bones after the skeleton is initialized
        StartCoroutine(WaitForSkeletonInit());
    }

    private System.Collections.IEnumerator WaitForSkeletonInit()
    {
        // Wait until the OVRSkeleton has finished initializing and the bones are populated
        while (handSkeleton == null || !handSkeleton.IsInitialized || handSkeleton.Bones.Count == 0)
        {
            yield return null;
        }

        // Loop through all bones in the skeleton
        foreach (var bone in handSkeleton.Bones)
        {
            OVRPlugin.BoneId boneId = (OVRPlugin.BoneId)bone.Id;

            // Only store bones within the valid bone ID range (skip Hand_Start and Hand_End)
            if (boneId >= OVRPlugin.BoneId.XRHand_Start && boneId <= OVRPlugin.BoneId.XRHand_End)
            {
                boneTransforms[boneId] = bone.Transform;
            }
        }

        Debug.Log("Hand bone transforms initialized.");
    }

    void Update()
    {
        // If bones haven't been initialized yet, do nothing
        if (boneTransforms.Count == 0) return;

        // Update all bone positions and rotations
        foreach (var kvp in boneTransforms)
        {
            OVRPlugin.BoneId boneId = kvp.Key;
            Transform boneTransform = kvp.Value;

            bonePositions[boneId] = boneTransform.position;
            boneRotations[boneId] = boneTransform.rotation;
        }

        // Update timer and log once per second
        logTimer += Time.deltaTime;
        if (logTimer >= 0.5f)
        {
            logTimer = 0f;

            // Check and log thumb tip data
            if (boneTransforms.ContainsKey(OVRPlugin.BoneId.XRHand_ThumbTip))
            {
                Transform thumbTip = boneTransforms[OVRPlugin.BoneId.XRHand_ThumbTip];
                Vector3 pos = thumbTip.position;
                Vector3 rot = thumbTip.rotation.eulerAngles;

                Debug.Log($"ThumbTip - Position: ({pos.x:F3}, {pos.y:F3}, {pos.z:F3}) | Rotation: ({rot.x:F1}, {rot.y:F1}, {rot.z:F1})");
            }
        }
    }
}
