using UnityEngine;

public class SimpleRobotJoint : MonoBehaviour
{
    public Vector3 rotateAxis;
    public Vector3 translateAxis;
    public float setpoint;
    public float maxVal;
    public float minVal;
    public float maxValSpeed = 0.1f;
    public float ValAccel = 0.5f;
    public float tolerance = 0.001f;

    private Quaternion initialRotation;
    private Vector3 initialTranslation;
    public float target;
    public float currVal;
    public float brakeDist;
    public float diff;
    public float currSpeed;
    public float currAccel;

    public int stage = 0;

    [HideInInspector]
    public Vector3 StartOffset;

    // Start is called before the first frame update
    void Start()
    {
        if(rotateAxis != rotateAxis.normalized)
        {
            Debug.LogWarning("Rotate Axis is not unit vector", this);
        }

        if (translateAxis != translateAxis.normalized)
        {
            Debug.LogWarning("Rotate Axis is not unit vector", this);
        }

        initialRotation = transform.localRotation;
        initialTranslation = transform.localPosition;

        StartOffset = transform.localPosition;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        target = Mathf.Clamp(setpoint, minVal, maxVal);

        diff = target - currVal;

        float sign = Mathf.Sign(diff);

        float dt = Time.fixedDeltaTime;

        brakeDist = currSpeed * currSpeed / (2 * ValAccel);

        if(Mathf.Abs(diff) < brakeDist && diff * currSpeed > 0 && (Mathf.Abs(diff) > tolerance || Mathf.Abs(currSpeed) > tolerance))
        {
            currAccel = -sign * ValAccel;
            stage = 3;
            //Debug.Log("Decelerating, diff=" + diff + " bdist=" +brakeDist + " accel=" + currAccel + " vel=" + currSpeed);
        }
        else if(Mathf.Abs(currSpeed) >= maxValSpeed && diff * currSpeed > 0)
        {
            currAccel = 0;
            stage = 2;
            //Debug.Log("Coasting, diff=" + diff + " bdist=" + brakeDist + " accel=" + currAccel + " vel=" + currSpeed);
        }
        else if (Mathf.Abs(diff) > tolerance || Mathf.Abs(currSpeed) > tolerance)
        {
            currAccel = sign * ValAccel;
            stage = 1;
            //Debug.Log("Accelerating, diff=" + diff + " bdist=" + brakeDist + " accel=" + currAccel + " vel=" + currSpeed);
        }
        else
        {
            currSpeed = 0;
            currAccel = 0;
            stage = 0;
        }

        currSpeed += currAccel * dt;
        currVal += currSpeed * dt;


        transform.localPosition = translateAxis * currVal + initialTranslation;
        transform.localRotation = Quaternion.Euler(rotateAxis * currVal * 360) * initialRotation;
    }
}
