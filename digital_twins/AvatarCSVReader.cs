using UnityEngine;
using System.Collections;
//using System.Diagnostics;


public class AvatarCSVReader : MonoBehaviour
{
    public TextAsset csvFile; // Reference to your .csv file
    public float scaleFactor = 1f; // Scaling factor for position values

    // Define the joints you have data for and their corresponding Transforms
    //public Transform neck_01;
    public Transform upperarm_l;
    public Transform lowerarm_l;
    public Transform hand_l;
    //public Transform upperarm_r;
    //public Transform lowerarm_r;
    //public Transform hand_r;

    void Start()
    {
        if (csvFile == null)
        {
            Debug.LogError("CSV file not assigned.");
            return;
        }

        if (string.IsNullOrWhiteSpace(csvFile.text))
        {
            Debug.LogError("CSV file is empty or not properly loaded.");
            return;
        }

        StartCoroutine(AnimateAvatar());
    }

    IEnumerator AnimateAvatar()
    {
        string[] lines = csvFile.text.Split('\n');

        if (lines.Length == 0)
        {
            Debug.LogError("No data in CSV file.");
            yield break;
        }

        foreach (string line in lines)
        {
            if (!string.IsNullOrWhiteSpace(line))
            {
                string[] data = line.Split(',');

                if (data.Length == 7)
                {
                    try
                    {
                        UpdateJointAngles(upperarm_l, lowerarm_l, hand_l, data);
                    }
                    catch (System.Exception ex)
                    {
                        Debug.LogError("Error parsing line: " + line + "\n" + ex);
                    }
                }
                else
                {
                    Debug.LogError("Incorrect data length in line: " + line);
                }

                yield return null; // Wait for the next frame
            }
        }
    }

    void UpdateJointAngles(Transform upperarm, Transform lowerarm, Transform hand, string[] data)
    {
        // Assuming angles are in degrees in the CSV file
        float upperarmX, upperarmY, upperarmZ, elbowY, handX, handY, handZ;

        // Check and parse each angle, handle empty or invalid data gracefully
        if (float.TryParse(data[0], out upperarmX))
            upperarmX *= scaleFactor;
        else
            upperarmX = 0f;

        if (float.TryParse(data[1], out upperarmY))
            upperarmY *= scaleFactor;
        else
            upperarmY = 0f;

        if (float.TryParse(data[2], out upperarmZ))
            upperarmZ *= scaleFactor;
        else
            upperarmZ = 0f;

        if (float.TryParse(data[3], out elbowY))
            elbowY *= scaleFactor;
        else
            elbowY = 0f;

        if (float.TryParse(data[4], out handX))
            handX *= scaleFactor;
        else
            handX = 0f;

        if (float.TryParse(data[5], out handY))
            handY *= scaleFactor;
        else
            handY = 0f;

        if (float.TryParse(data[6], out handZ))
            handZ *= scaleFactor;
        else
            handZ = 0f;

        // Set the rotations for the left arm joints
        upperarm.localEulerAngles = new Vector3(upperarmX, upperarmY, upperarmZ);
        lowerarm.localEulerAngles = new Vector3(0f, elbowY, 0f); // Assuming local rotation for elbow
        hand.localEulerAngles = new Vector3(handX, handY, handZ);

        // Log the new angles for the left arm
        Debug.Log("Left Arm angles set - Upperarm: " + upperarm.localEulerAngles.ToString("F6") +
                  ", Elbow: " + lowerarm.localEulerAngles.ToString("F6") +
                  ", Hand: " + hand.localEulerAngles.ToString("F6"));
    }

}
