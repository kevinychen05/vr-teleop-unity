using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;
using System.Collections.Generic;

public class HandSkeletonPublisher : MonoBehaviour
{
    public OVRSkeleton skeleton;
    public string topicName = "hand_skeleton";
    public float publishRate = 0.05f; // ~20 Hz

    private ROSConnection ros;
    private float timeElapsed = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<HandSkeletonMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishRate && skeleton.IsDataValid)
        {
            List<int> boneIds = new List<int>();
            List<PosRotMsg> poses = new List<PosRotMsg>();

            foreach (var bone in skeleton.Bones)
            {
                boneIds.Add((int)bone.Id);
                var tf = bone.Transform;

                poses.Add(new PosRotMsg(
                    tf.position.x, tf.position.y, tf.position.z,
                    tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
                ));
            }

            var msg = new HandSkeletonMsg
            {
                bone_ids = boneIds.ToArray(),
                bone_poses = poses.ToArray()
            };

            ros.Publish(topicName, msg);
            timeElapsed = 0f;
        }
    }
}
