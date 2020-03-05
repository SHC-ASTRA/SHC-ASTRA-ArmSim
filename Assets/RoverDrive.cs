using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RoverDrive : MonoBehaviour
{
    public List<WheelCollider> Wheels;
    public float maxMotorTorque;
    public float steerTorqueFraction;

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    public void FixedUpdate()
    {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = steerTorqueFraction * Input.GetAxis("Horizontal");

        foreach (WheelCollider wheel in Wheels)
        {
            wheel.motorTorque = motor + (wheel.transform.localPosition.x > 0 ? -1 : 1) * steering;
            
            ApplyLocalPositionToVisuals(wheel);
        }
    }
}