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

    private NelderMeadSimplex solver;

    private JobHandle ikJobHandle;
    private NativeArray<double> anglesNative;
    private NativeArray<float> angleMaxesNative;
    private NativeArray<float> angleMinsNative;
    private NativeArray<Vector3> rotAxesNative;
    private NativeArray<Vector3> transAxesNative;
    private NativeArray<Vector3> startOffsetsNative;
    private bool runOnce = false;

    void OnApplicationQuit()
    {
        ikJobHandle.Complete();

        anglesNative.Dispose();
        angleMaxesNative.Dispose();
        angleMinsNative.Dispose();
        rotAxesNative.Dispose();
        transAxesNative.Dispose();
        startOffsetsNative.Dispose();
    }

    void Start()
    {
        angles = new double[Joints.Length];
        angleMins = new float[Joints.Length];
        angleMaxes = new float[Joints.Length];
        rotAxes = new Vector3[Joints.Length];
        transAxes = new Vector3[Joints.Length];
        startOffsets = new Vector3[Joints.Length];
        for (int i = 0; i < Joints.Length; i++)
        {
            angleMins[i] = Joints[i].GetComponent<SimpleRobotJoint>().minVal * 360;
            angleMaxes[i] = Joints[i].GetComponent<SimpleRobotJoint>().maxVal * 360;
            rotAxes[i] = Joints[i].GetComponent<SimpleRobotJoint>().rotateAxis;
            transAxes[i] = Joints[i].GetComponent<SimpleRobotJoint>().translateAxis;
            startOffsets[i] = Joints[i].GetComponent<SimpleRobotJoint>().StartOffset;
        }

        solver = new NelderMeadSimplex(convergenceTolerance, MaxIterations);

        ikJobHandle = new JobHandle();
    }

    // Update is called once per frame
    void Update()
    {
        if(Joints.Length == 0)
        {
            return;
        }
        for(int i = 0; i < Joints.Length; i++)
        {
            Joints[i].GetComponent<SimpleRobotJoint>().setpoint = (float) angles[i]/360f;
            startOffsets[i] = Joints[i].GetComponent<SimpleRobotJoint>().StartOffset;
        }
        for(int i = 0; i < Grips.Length; i++) 
        {
            Grips[i].GetComponent<SimpleRobotJoint>().setpoint = grip;
        }

        Vector3 finalPoint;
        Quaternion finalRot;

        ForwardKinematicsDraw(angles, Joints[0].transform.position, Joints[0].transform.rotation, rotAxes, transAxes, startOffsets, out finalPoint, out finalRot);
        Debug.DrawLine(finalPoint - finalRot * Vector3.left * 0.1f, finalPoint + finalRot * Vector3.left * 0.1f, Color.red);
        Debug.DrawLine(finalPoint - finalRot * Vector3.forward * 0.1f, finalPoint + finalRot * Vector3.forward * 0.1f, Color.blue);
        Debug.DrawLine(finalPoint - finalRot * Vector3.up * 0.1f, finalPoint + finalRot * Vector3.up * 0.1f, Color.green);

        if(!runOnce || (runOnce && ikJobHandle.IsCompleted))
        {
            ikJobHandle.Complete();

            if (runOnce && ikJobHandle.IsCompleted)
            {
                anglesNative.CopyTo(angles);

                for(int i = 0; i < angles.Length; i++)
                {
                    angles[i] = (double)Mathf.Clamp((float)angles[i], angleMins[i], angleMaxes[i]); 
                }

                anglesNative.Dispose();
                angleMaxesNative.Dispose();
                angleMinsNative.Dispose();
                rotAxesNative.Dispose();
                transAxesNative.Dispose();
                startOffsetsNative.Dispose();
            }

            runOnce = true;


            anglesNative = new NativeArray<double>(angles.Length, Allocator.TempJob);
            anglesNative.CopyFrom(angles);

            angleMaxesNative = new NativeArray<float>(angleMaxes.Length, Allocator.TempJob);
            angleMaxesNative.CopyFrom(angleMaxes);

            angleMinsNative = new NativeArray<float>(angleMins.Length, Allocator.TempJob);
            angleMinsNative.CopyFrom(angleMins);

            rotAxesNative = new NativeArray<Vector3>(rotAxes.Length, Allocator.TempJob);
            rotAxesNative.CopyFrom(rotAxes);

            transAxesNative = new NativeArray<Vector3>(transAxes.Length, Allocator.TempJob);
            transAxesNative.CopyFrom(transAxes);

            startOffsetsNative = new NativeArray<Vector3>(startOffsets.Length, Allocator.TempJob);
            startOffsetsNative.CopyFrom(startOffsets);

            //Debug.Log(result.FunctionInfoAtMinimum.Value);

            IKJob ikJob = new IKJob
            {
                targetPos = targetTransform.position,
                targetRot = targetTransform.rotation,
                jobAngles = anglesNative,
                jobAngleMins = angleMinsNative,
                jobAngleMaxes = angleMaxesNative,
                jobConvergenceTolerance = convergenceTolerance,
                jobDistanceThreshold = DistanceThreshold,
                jobMaxIterations = MaxIterations,
                jobBasePos = Joints[0].transform.position,
                jobBaseRot = Joints[0].transform.rotation,
                jobRotAxes = rotAxesNative,
                jobTransAxes = transAxesNative,
                jobStartOffsets = startOffsetsNative
            };

            ikJobHandle = ikJob.Schedule();

            JobHandle.ScheduleBatchedJobs();
        }

       




        //for (int i = 0; i < 100; i++)
        //{
        //    InverseKinematics(targetTransform.position, targetTransform.rotation, angles);
        //}

        //Debug.Log("Current Error: " + ErrorFunction(targetTransform.position, targetTransform.rotation, angles));


    }

    static void ForwardKinematics(double[] deltas, Vector3 basePos, Quaternion baseRot, Vector3[] rotAxes, Vector3[] transAxes, Vector3[] startOffsets, out Vector3 finalPoint, out Quaternion finalRotation)
    {
        Vector3 prevPoint = basePos;
        Quaternion rotation = baseRot;
        for (int i = 1; i < deltas.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis((float)deltas[i - 1], rotAxes[i - 1]);
            Vector3 nextPoint = prevPoint + rotation * startOffsets[i] + rotation * transAxes[i] * ((float)deltas[i]/360f);

            prevPoint = nextPoint;
        }

        finalPoint = prevPoint;
        rotation *= Quaternion.AngleAxis((float)deltas[deltas.Length - 1], rotAxes[deltas.Length - 1]);
        finalRotation = rotation;
    }

    static void ForwardKinematicsDraw(double[] deltas, Vector3 basePos, Quaternion baseRot, Vector3[] rotAxes, Vector3[] transAxes, Vector3[] startOffsets, out Vector3 finalPoint, out Quaternion finalRotation)
    {
        Color[] colorSequence = { Color.white, Color.gray };

        Vector3 prevPoint = basePos;
        Quaternion rotation = baseRot;
        for (int i = 1; i < deltas.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis((float)deltas[i - 1], rotAxes[i - 1]);
            Vector3 nextPoint = prevPoint + rotation * startOffsets[i] + rotation * transAxes[i] * ((float)deltas[i]/360f);

            Debug.DrawLine(prevPoint, nextPoint, colorSequence[i % colorSequence.Length]);

            prevPoint = nextPoint;
        }

        finalPoint = prevPoint;
        rotation *= Quaternion.AngleAxis((float)deltas[deltas.Length - 1], rotAxes[deltas.Length - 1]);
        finalRotation = rotation;
    }

    static void DistanceAndRotationFromTarget(Vector3 target, Quaternion targetRot, double[] angles, float[] minAngles, float[] maxAngles, Vector3 basePos, Quaternion baseRot, Vector3[] rotAxes, Vector3[] transAxes, Vector3[] startOffsets, out float distance, out float rotationDist)
    {
        float penalty = 0;
        for (int i = 0; i < angles.Length; i++)
        {
            if (angles[i] < minAngles[i] || angles[i] > maxAngles[i])
            {
                penalty += 1;
            }
            angles[i] = (double) Mathf.Clamp((float)angles[i], minAngles[i], maxAngles[i]);
        }
        Vector3 point;
        Quaternion rotation;
        ForwardKinematics(angles, basePos, baseRot, rotAxes, transAxes, startOffsets, out point, out rotation);

        distance = Vector3.Distance(point, target) + penalty;
        rotationDist = Mathf.Abs
        (
             Quaternion.Angle(rotation, targetRot) / 180f
        );
    }


    public static float ErrorFunction(Vector3 target, Quaternion targetRot, double[] angles, float[] minAngles, float[] maxAngles, Vector3 basePos, Quaternion baseRot, Vector3[] rotAxes, Vector3[] transAxes, Vector3[] startOffsets)
    {
        float distancePenalty; 
        float rotationPenalty;

        DistanceAndRotationFromTarget(target, targetRot, angles, minAngles, maxAngles, basePos, baseRot, rotAxes, transAxes, startOffsets, out distancePenalty, out rotationPenalty);

        return
            distancePenalty+
            rotationPenalty*3f;
    }


}

public struct IKJob : IJob
{
    public Vector3 targetPos;
    public Quaternion targetRot;
    public NativeArray<double> jobAngles;
    public NativeArray<float> jobAngleMins;
    public NativeArray<float> jobAngleMaxes;
    public float jobConvergenceTolerance;
    public float jobDistanceThreshold;
    public int jobMaxIterations;
    public Vector3 jobBasePos;
    public Quaternion jobBaseRot;
    public NativeArray<Vector3> jobRotAxes;
    public NativeArray<Vector3> jobTransAxes;
    public NativeArray<Vector3> jobStartOffsets;

    public void Execute()
    {
        NelderMeadSimplex solver = new NelderMeadSimplex(jobConvergenceTolerance, jobMaxIterations);
        
        var V = Vector<double>.Build;
        var meJob = this;
        var f1 = new Func<Vector<double>, double>(angleVec => RobotController.ErrorFunction(meJob.targetPos, meJob.targetRot, angleVec.ToArray(), meJob.jobAngleMins.ToArray(), meJob.jobAngleMaxes.ToArray(), meJob.jobBasePos, meJob.jobBaseRot, meJob.jobRotAxes.ToArray(), meJob.jobTransAxes.ToArray(), meJob.jobStartOffsets.ToArray()));
        var obj = ObjectiveFunction.Value(f1);

        MinimizationResult result;
        try
        {
            result = solver.FindMinimum(obj, V.DenseOfArray(jobAngles.ToArray()));

            for (int i = 0; i < 5; i++)
            {
                if (result.FunctionInfoAtMinimum.Value > jobDistanceThreshold)
                {
                    f1 = new Func<Vector<double>, double>(angleVec => RobotController.ErrorFunction( meJob.targetPos, meJob.targetRot, angleVec.ToArray(), meJob.jobAngleMins.ToArray(), meJob.jobAngleMaxes.ToArray(), meJob.jobBasePos, meJob.jobBaseRot, meJob.jobRotAxes.ToArray(), meJob.jobTransAxes.ToArray(), meJob.jobStartOffsets.ToArray()));
                    obj = ObjectiveFunction.Value(f1);
                    MinimizationResult result2;
                    try
                    {
                        result2 = solver.FindMinimum(obj, V.DenseOfArray(jobAngles.ToArray()) + (V.Random(jobAngles.Length) * (i + 1) * (i + 1) * (i + 1)));
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