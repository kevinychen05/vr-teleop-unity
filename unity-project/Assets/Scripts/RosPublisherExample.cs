using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;
using System.Linq;

public class RosPublisherExample : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";
    public OVRSkeleton skeleton; // Assign OVRHandPrefab with OVRSkeleton in Inspector
    public float publishMessageFrequency = 0.02f;

    private float timeElapsed;
    private Transform thumbTip;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);

        // Find the thumb tip bone at runtime
        thumbTip = skeleton.Bones
            .FirstOrDefault(b => b.Id == OVRSkeleton.BoneId.Hand_Thumb3)?.Transform;

        if (thumbTip == null)
        {
            Debug.LogWarning("Thumb tip not found!");
        }
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency && thumbTip != null)
        {
            Vector3 pos = thumbTip.position;
            Quaternion rot = thumbTip.rotation;

            PosRotMsg msg = new PosRotMsg(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w);
            ros.Publish(topicName, msg);

            timeElapsed = 0;
        }
    }
}

// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.UnityRoboticsDemo;

// /// <summary>
// ///
// /// </summary>
// public class RosPublisherExample : MonoBehaviour
// {
//     ROSConnection ros;
//     public string topicName = "pos_rot";

//     // The game object
//     public GameObject cube;
//     // Publish the cube's position and rotation every N seconds
//     public float publishMessageFrequency = 0.5f;

//     // Used to determine how much time has elapsed since the last message was published
//     private float timeElapsed;

//     void Start()
//     {
//         // start the ROS connection
//         ros = ROSConnection.GetOrCreateInstance();
//         ros.RegisterPublisher<PosRotMsg>(topicName);
//     }

//     private void Update()
//     {
//         timeElapsed += Time.deltaTime;

//         if (timeElapsed > publishMessageFrequency)
//         {
//             cube.transform.rotation = Random.rotation;

//             PosRotMsg cubePos = new PosRotMsg(
//                 cube.transform.position.x,
//                 cube.transform.position.y,
//                 cube.transform.position.z,
//                 cube.transform.rotation.x,
//                 cube.transform.rotation.y,
//                 cube.transform.rotation.z,
//                 cube.transform.rotation.w
//             );

//             // Finally send the message to server_endpoint.py running in ROS
//             ros.Publish(topicName, cubePos);

//             timeElapsed = 0;
//         }
//     }
// }