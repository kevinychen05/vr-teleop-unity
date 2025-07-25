using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CubePoseSubscriber : MonoBehaviour
{
    public GameObject cubeToMove;

    // Optional offsets and scale if needed (tweak in Inspector)
    public Vector3 positionOffset = new Vector3(-1.0f, -0.5f, -2.0f);
    public float scaleFactor = 1.0f;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>(
            "/cube_pose", OnReceivePose);
    }

    void OnReceivePose(PoseMsg msg)
    {
        // Convert position from ROS (ENU) to Unity (Y-up, left-handed)
        Vector3 rosPosition = new Vector3(
            (float)msg.position.x,
            (float)msg.position.y,
            (float)msg.position.z
        );

        // Convert to Unity coordinates (x, z, y) + optional scale/offset
        Vector3 unityPosition = new Vector3(
            -rosPosition.y,
            rosPosition.z, // ROS Z -> Unity Y
            rosPosition.x  // ROS Y -> Unity Z
        ) * scaleFactor + positionOffset;

        // Convert quaternion from ROS to Unity (adjust axes if needed)
        Quaternion rosRotation = new Quaternion(
            (float)msg.orientation.x,
            (float)msg.orientation.y,
            (float)msg.orientation.z,
            (float)msg.orientation.w
        );

        // Optionally adjust rotation mapping if it's visibly wrong (swap Y/Z for example)
        Quaternion unityRotation = new Quaternion(
            rosRotation.x,
            rosRotation.z, // ROS Z -> Unity Y
            rosRotation.y, // ROS Y -> Unity Z
            rosRotation.w
        );

        // Apply position and rotation
        cubeToMove.transform.SetPositionAndRotation(unityPosition, unityRotation);
    }
}
