using UnityEngine;

public class BoneFollower : MonoBehaviour
{
    // Assign this in the Inspector to the HandBoneTracker component in your scene
    public BoneTracking boneTracker;

    // Which bone to follow — default is Thumb Tip
    public OVRPlugin.BoneId targetBone = OVRPlugin.BoneId.XRHand_ThumbTip;

    void Update()
    {
        // Make sure bone data exists
        if (boneTracker == null || !boneTracker.boneTransforms.ContainsKey(targetBone)) return;

        // Get the transform of the bone
        Transform boneTransform = boneTracker.boneTransforms[targetBone];

        // Set this GameObject's position and rotation to match the bone
        transform.position = boneTransform.position;
        transform.rotation = boneTransform.rotation;
    }
}