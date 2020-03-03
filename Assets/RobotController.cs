using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.LinearAlgebra;

public class RobotController : MonoBehaviour
{
    public GameObject[] Joints;
    public float[] angleMins;
    public float[] angleMaxes;
    public Vector3[] rotAxes;
    public Vector3[] transAxes;
    public Vector3[] startOffsets;
    public GameObject[] Grips;
    public double[] angles;

    public float convergenceTolerance;
    public int MaxIterations;

    public float DistanceThreshold;



    public Transform targetTransform;


    [Range(-1.0f, 1.0f)]
    public float grip;

    private Color[] colorSequence = { Color.white, Color.gray};

    private NelderMeadSimplex solver;

    void Start()
    {
        angles = new double[Joints.Length];
        angleMins = new float[Joints.Length];
        angleMaxes = new float[Joints.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            angleMins[i] = Joints[i].GetComponent<SimpleRobotJoint>().minVal * 360;
            angleMaxes[i] = Joints[i].GetComponent<SimpleRobotJoint>().maxVal * 360;
        }

        solver = new NelderMeadSimplex(convergenceTolerance, MaxIterations);
    }

    // Update is called once per frame
    void Update()
    {
        for(int i = 0; i < Joints.Length; i++)
        {
            Joints[i].GetComponent<SimpleRobotJoint>().setpoint = (float) angles[i]/360f;
        }
        Grips[0].GetComponent<SimpleRobotJoint>().setpoint = grip;
        Grips[1].GetComponent<SimpleRobotJoint>().setpoint = grip;

        Vector3 finalPoint = ForwardKinematicsDraw(angles);
        Debug.DrawLine(finalPoint - Vector3.left * 0.1f, finalPoint + Vector3.left * 0.1f, Color.red);
        Debug.DrawLine(finalPoint - Vector3.forward * 0.1f, finalPoint + Vector3.forward * 0.1f, Color.blue);
        Debug.DrawLine(finalPoint - Vector3.up * 0.1f, finalPoint + Vector3.up * 0.1f, Color.green);


        var anglesNative = new NativeArray<double>(angles.Length, Allocator.Temp);
        anglesNative.CopyFrom(angles);

        //Debug.Log(result.FunctionInfoAtMinimum.Value);

        IKJob ikJob = new IKJob
        {
            targetPos = targetTransform.position,
            targetRot = targetTransform.rotation,
            jobAngles = anglesNative,
            robotController = this
        };

        var ikJobHandle = ikJob.Schedule();

        JobHandle.ScheduleBatchedJobs();

        ikJobHandle.Complete();

        anglesNative.CopyTo(angles);

        anglesNative.Dispose();
        
        

        //for (int i = 0; i < 100; i++)
        //{
        //    InverseKinematics(targetTransform.position, targetTransform.rotation, angles);
        //}

        //Debug.Log("Current Error: " + ErrorFunction(targetTransform.position, targetTransform.rotation, angles));
        

    }

    void ForwardKinematics(double[] angles, out Vector3 finalPoint, out Quaternion finalRotation)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis((float)angles[i - 1], Joints[i - 1].GetComponent<SimpleRobotJoint>().rotateAxis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].GetComponent<SimpleRobotJoint>().StartOffset;

            prevPoint = nextPoint;
        }

        finalPoint = prevPoint;
        rotation *= Quaternion.AngleAxis((float)angles[Joints.Length - 1], Joints[Joints.Length - 1].GetComponent<SimpleRobotJoint>().rotateAxis);
        finalRotation = rotation;
    }

    
    public Vector3 ForwardKinematicsDraw(double[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis((float)angles[i - 1], Joints[i - 1].GetComponent<SimpleRobotJoint>().rotateAxis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].GetComponent<SimpleRobotJoint>().StartOffset;

            Debug.DrawLine(prevPoint, nextPoint, colorSequence[i%colorSequence.Length]);

            prevPoint = nextPoint;
        }
        return prevPoint;
    }

    public float DistanceFromTarget(Vector3 target, double[] angles)
    {
        Vector3 point;
        Quaternion rotation;
        ForwardKinematics(angles, out point, out rotation);
        return Vector3.Distance(point, target);
    }

    public float RotationFromTarget(Quaternion target, double[] angles)
    {
        Vector3 point;
        Quaternion rotation;
        ForwardKinematics(angles, out point, out rotation);
        return Mathf.Abs
        (
             Quaternion.Angle(rotation, target) / 180f
        );
    }

    public void DistanceAndRotationFromTarget(Vector3 target, Quaternion targetRot, double[] angles, float[] minAngles, float[] maxAngles, out float distance, out float rotationDist)
    {
        float penalty = 0;
        for (int i = 0; i < angles.Length; i++)
        {
            if (angles[i] < minAngles[i] || angles[i] > maxAngles[i])
            {
                penalty += 1;
            }
            angles[i] = Mathf.Clamp((float)angles[i], minAngles[i], maxAngles[i]);
        }
        Vector3 point;
        Quaternion rotation;
        ForwardKinematics(angles, out point, out rotation);

        distance = Vector3.Distance(point, target) + penalty;
        rotationDist = Mathf.Abs
        (
             Quaternion.Angle(rotation, targetRot) / 180f
        );
    }


    public static float ErrorFunction(RobotController robotController, Vector3 target, Quaternion targetRot, double[] angles, float[] minAngles, float[] maxAngles)
    {
        float distancePenalty; 
        float rotationPenalty;

        robotController.DistanceAndRotationFromTarget(target, targetRot, angles, minAngles, maxAngles, out distancePenalty, out rotationPenalty);

        return
            distancePenalty / 2f *1f+
            rotationPenalty * 1f;
    }

    

    //public float PartialGradient(Vector3 target, Quaternion targetRot, double[] angles, int i)
    //{
    //    // Saves the angle,
    //    // it will be restored later
    //    float angle = (float) angles[i];

    //    // Gradient : [F(x+SamplingDistance) - F(x)] / h
    //    float f_x = ErrorFunction(target, targetRot, angles);

    //    angles[i] += SamplingDistance;
    //    float f_x_plus_d = ErrorFunction(target, targetRot, angles);

    //    float gradient = (f_x_plus_d - f_x) / SamplingDistance;

    //    // Restores
    //    angles[i] = angle;

    //    return gradient;
    //}

    //public void InverseKinematics(Vector3 target, Quaternion targetRot, double[] angles)
    //{
    //    if (ErrorFunction(target, targetRot, angles) < DistanceThreshold)
    //        return;

    //    for (int i = Joints.Length - 1; i >= 0; i--)
    //    {
    //        // Gradient descent
    //        // Update : Solution -= LearningRate * Gradient
    //        float gradient = PartialGradient(target, targetRot, angles, i);
    //        angles[i] -= LearningRate * gradient;

    //        angles[i] = Mathf.Clamp((float)angles[i], Joints[i].GetComponent<SimpleRobotJoint>().minVal * 360, Joints[i].GetComponent<SimpleRobotJoint>().maxVal * 360);

    //        if (ErrorFunction(target, targetRot, angles) < DistanceThreshold)
    //            return;
    //    }
    //}
}

public struct IKJob : IJob
{
    public Vector3 targetPos;
    public Quaternion targetRot;
    public NativeArray<double> jobAngles;
    public RobotController robotController;

    public void Execute()
    {

        float[] jobAngleMins = robotController.angleMins;
        float[] jobAngleMaxes = robotController.angleMaxes;
        float jobConvergenceTolerance = robotController.convergenceTolerance;
        float jobDistanceThreshold = robotController.DistanceThreshold;
        int jobMaxIterations = robotController.MaxIterations;

        NelderMeadSimplex solver = new NelderMeadSimplex(jobConvergenceTolerance, jobMaxIterations);

        var V = Vector<double>.Build;
        var meJob = this;
        var f1 = new Func<Vector<double>, double>(angleVec => RobotController.ErrorFunction(meJob.robotController, meJob.targetPos, meJob.targetRot, angleVec.ToArray(), jobAngleMins, jobAngleMaxes));
        var obj = ObjectiveFunction.Value(f1);

        MinimizationResult result;
        try
        {
            result = solver.FindMinimum(obj, V.DenseOfArray(jobAngles.ToArray()));

            for (int i = 0; i < 3; i++)
            {
                if (result.FunctionInfoAtMinimum.Value > jobDistanceThreshold)
                {
                    f1 = new Func<Vector<double>, double>(angleVec => RobotController.ErrorFunction(meJob.robotController, meJob.targetPos, meJob.targetRot, angleVec.ToArray(), jobAngleMins, jobAngleMaxes));
                    obj = ObjectiveFunction.Value(f1);
                    MinimizationResult result2;
                    try
                    {
                        result2 = solver.FindMinimum(obj, V.DenseOfArray(jobAngles.ToArray()) + (V.Random(6) * (i + 1) * 20));
                        if (result2 != null && result2.FunctionInfoAtMinimum.Value < result.FunctionInfoAtMinimum.Value)
                        {
                            result = result2;
                        }
                    }
                    catch (MaximumIterationsException e)
                    {
                        Debug.Log(e);
                    }
                }
                else
                {
                    break;
                }
            }

            jobAngles.CopyFrom(result.MinimizingPoint.ToArray());
        }
        catch (MaximumIterationsException e)
        {
            Debug.Log(e);
        }

    }
}