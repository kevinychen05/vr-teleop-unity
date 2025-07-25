using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class PointCloudSubscriber : MonoBehaviour
{
    public GameObject pointPrefab;  // Assign a small sphere/cube prefab in the Inspector
    public int maxPoints = 500;     // Maximum points to visualize
    public float scaleFactor = 1.0f; // Optional scaling factor

    private List<GameObject> spawnedPoints = new List<GameObject>();

    void Start()
    {
        // Subscribe to the /depth/points topic
        ROSConnection.GetOrCreateInstance().Subscribe<PointCloud2Msg>(
            "/camera/depth/points", OnReceivePointCloud);
    }

    void OnReceivePointCloud(PointCloud2Msg msg)
    {
        Debug.Log("Point received");
        Debug.Log($"Width: {msg.width}, PointStep: {msg.point_step}, Data length: {msg.data.Length}");

        // Clear previously spawned points
        foreach (GameObject go in spawnedPoints)
        {
            Destroy(go);
        }
        spawnedPoints.Clear();

        // Get point step (number of bytes per point)
        int pointStep = (int)msg.point_step;
        int offsetX = GetFieldOffset(msg.fields, "x");
        int offsetY = GetFieldOffset(msg.fields, "y");
        int offsetZ = GetFieldOffset(msg.fields, "z");

        if (offsetX < 0 || offsetY < 0 || offsetZ < 0)
        {
            Debug.LogWarning("One or more fields not found in PointCloud2");
            return;
        }

        int spawnedCount = 0;

        for (int i = 0; i < msg.width && spawnedCount < maxPoints; i++)
        {
            int index = i * pointStep;

            float x = BitConverter.ToSingle(msg.data, index + offsetX);
            float y = BitConverter.ToSingle(msg.data, index + offsetY);
            float z = BitConverter.ToSingle(msg.data, index + offsetZ);

            if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || z <= 0.001f)
            {
                continue;
            }

            if (spawnedCount < 5)
            {
                Debug.Log($"Spawned Point {spawnedCount}: x = {x}, y = {y}, z = {z}");
            }

            Vector3 position = new Vector3(x, y, z) * scaleFactor;
            GameObject point = Instantiate(pointPrefab, position, Quaternion.identity, this.transform);
            spawnedPoints.Add(point);
            spawnedCount++;
        }
    }

    int GetFieldOffset(PointFieldMsg[] fields, string fieldName)
    {
        foreach (PointFieldMsg field in fields)
        {
            if (field.name == fieldName)
                return (int)field.offset;
        }
        return -1; // Field not found
    }
}
