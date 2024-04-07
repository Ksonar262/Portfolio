using UnityEngine;
using System.Collections;

public class PosFileReader : MonoBehaviour
{
    public TextAsset csvFile; // Reference to your .csv file
    public float scaleFactor = 1f; // Scaling factor for position values

    // Define the joints you have data for and their corresponding Transforms
    public Transform clavicle_l;
    public Transform clavicle_r;
    public Transform shoulder_l;
    public Transform shoulder_r;
    public Transform elbow_l;
    public Transform elbow_r;
    public Transform wrist_l;
    public Transform wrist_r;
    public Transform hand_l;
    public Transform hand_r;

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

                if (data.Length >= 38)
                {
                    try
                    {
                        //UpdateJointRotation(clavicle_l, data, 0);
                        //UpdateJointRotation(clavicle_r, data, 4);
                        //UpdateJointRotation(shoulder_l, data, 8);
                        //UpdateJointRotation(shoulder_r, data, 12);
                        //UpdateJointRotation(elbow_l, data, 16);
                        //UpdateJointRotation(elbow_r, data, 20);
                        //UpdateJointRotation(wrist_l, data, 24);
                        UpdateJointPosition(hand_l, data, 32);
                        //UpdateJointRotation(wrist_r, data, 28);
                        UpdateJointPosition(hand_r, data, 35);
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

    void UpdateJointPosition(Transform joint, string[] data, int startIndex)
    {
        double x = double.Parse(data[startIndex]) * scaleFactor;
        double y = double.Parse(data[startIndex + 1]) * scaleFactor;
        double z = double.Parse(data[startIndex + 2]) * scaleFactor;

        Vector3 newPosition = new Vector3((float)x, (float)y, (float)z);
        joint.localPosition = newPosition;

        //Log the new position
        Debug.Log(joint.name + " position set to: " + newPosition.ToString("F6"));
    }

    void UpdateJointRotation(Transform joint, string[] data, int startIndex)
    {
        double xRot = double.Parse(data[startIndex]);
        double yRot = double.Parse(data[startIndex + 1]);
        double zRot = double.Parse(data[startIndex + 2]);
        double wRot = double.Parse(data[startIndex + 3]);

        Quaternion newRotation = new Quaternion((float)xRot, (float)yRot, (float)zRot, (float)wRot);
        joint.localRotation = newRotation;

        //Log the new rotation
        Debug.Log(joint.name + " rotation set to: " + newRotation.eulerAngles.ToString("F6"));
    }

}
