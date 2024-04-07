using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class HandPositionUpdater : MonoBehaviour
{
    // Reference to the avatar's root transform
    public Transform avatarRoot;

    void Start()
    {
        // Read CSV file
        string[] lines = File.ReadAllLines("E:\\Twinality\\Twinality");

        // Find the left hand joint based on your avatar's hierarchy
        Transform leftHand = FindChildTransform(avatarRoot, "hand_l");

        foreach (var line in lines)
        {
            // Parse CSV data (assuming CSV format: X,Y,Z)
            string[] values = line.Split(',');
            float x = float.Parse(values[0]);
            float y = float.Parse(values[1]);
            float z = float.Parse(values[2]);

            // Update left hand position
            if (leftHand != null)
            {
                leftHand.position = new Vector3(x, y, z);
            }
        }
    }

    // Recursive function to find a child transform by name
    Transform FindChildTransform(Transform parent, string childName)
    {
        foreach (Transform child in parent)
        {
            if (child.name == childName)
            {
                return child;
            }

            Transform foundChild = FindChildTransform(child, childName);
            if (foundChild != null)
            {
                return foundChild;
            }
        }

        return null;
    }
}
