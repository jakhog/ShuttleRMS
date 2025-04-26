using System;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class Kinematics
{
    private readonly Vector3 _shoulderYawAxis;
    private readonly Vector3 _shoulderYawOffset;
    private readonly Vector3 _shoulderPitchAxis;
    private readonly Vector3 _shoulderPitchOffset;
    private readonly Vector3 _elbowPitchAxis;
    private readonly Vector3 _elbowPitchOffset;
    private readonly Vector3 _wristPitchAxis;
    private readonly Vector3 _wristPitchOffset;
    private readonly Vector3 _wristYawAxis;
    private readonly Vector3 _wristYawOffset;
    private readonly Vector3 _wristRollAxis;
    private readonly Vector3 _wristRollOffset;

    private readonly Matrix4x4 _endEffectorTransform;

    private Matrix4x4 _shoulderYawTransform;
    private Matrix4x4 _shoulderPitchTransform;
    private Matrix4x4 _elbowPitchTransform;
    private Matrix4x4 _wristPitchTransform;
    private Matrix4x4 _wristYawTransform;
    private Matrix4x4 _wristRollTransform;

    private float _shoulderYawAngle = 0;
    private float _shoulderPitchAngle = 0;
    private float _elbowPitchAngle = 0;
    private float _wristPitchAngle = 0;
    private float _wristYawAngle = 0;
    private float _wristRollAngle = 0;

    public Kinematics(
		Vector3 shoulderYawAxis, Vector3 shoulderYawOffset,
		Vector3 shoulderPitchAxis, Vector3 shoulderPitchOffset,
		Vector3 elbowPitchAxis, Vector3 elbowPitchOffset,
        Vector3 wristPitchAxis, Vector3 wristPitchOffset,
        Vector3 wristYawAxis, Vector3 wristYawOffset,
        Vector3 wristRollAxis, Vector3 wristRollOffset,
        Vector3 endEffectorOffset
    )
	{
        _shoulderYawAxis = shoulderYawAxis;
        _shoulderYawOffset = shoulderYawOffset;
        _shoulderPitchAxis = shoulderPitchAxis;
        _shoulderPitchOffset = shoulderPitchOffset;
        _elbowPitchAxis = elbowPitchAxis;
        _elbowPitchOffset = elbowPitchOffset;
        _wristPitchAxis = wristPitchAxis;
        _wristPitchOffset = wristPitchOffset;
        _wristYawAxis = wristYawAxis;
        _wristYawOffset = wristYawOffset;
        _wristRollAxis = wristRollAxis;
        _wristRollOffset = wristRollOffset;

        _endEffectorTransform = Matrix4x4.Translate(endEffectorOffset);

        RecalculateTransforms();
    }

    public Matrix4x4 ArmTransform { get; private set; }

    public void SetAngles(
        float shoulderYaw = 0,
        float shoulderPitch = 0,
        float elbowPitch = 0,
        float wristPitch = 0,
        float wristYaw = 0,
        float wristRoll = 0
    )
    {
        _shoulderYawAngle = shoulderYaw;
        _shoulderPitchAngle = shoulderPitch;
        _elbowPitchAngle = elbowPitch;
        _wristPitchAngle = wristPitch;
        _wristYawAngle = wristYaw;
        _wristRollAngle = wristRoll;

        RecalculateTransforms();
    }

    public void SetAngles(Vector<float> angles)
    {
        if (angles.Count > 0) _shoulderYawAngle = angles[0];
        if (angles.Count > 1) _shoulderPitchAngle = angles[1];
        if (angles.Count > 2) _elbowPitchAngle = angles[2];
        if (angles.Count > 3) _wristPitchAngle = angles[3];
        if (angles.Count > 4) _wristYawAngle = angles[4];
        if (angles.Count > 5) _wristRollAngle = angles[5];

        RecalculateTransforms();
    }


    public void FiniteDiffJacobian(Matrix<float> jacobian, Vector<float> origin, Vector<float> pertubed, Action<Matrix4x4, Vector<float>> decompose, float delta = 0.001f)
    {
        decompose(ArmTransform, origin);

        // ShoulderYaw
        var transform =
            Matrix4x4.TRS(
                _shoulderYawOffset,
                Quaternion.AngleAxis(_shoulderYawAngle + delta, _shoulderYawAxis),
                Vector3.one
            )
            * _shoulderPitchTransform
            * _elbowPitchTransform
            * _wristPitchTransform
            * _wristYawTransform
            * _wristRollTransform;
        decompose(transform, pertubed);
        jacobian.SetColumn(0, (pertubed - origin) / delta);

        // ShoulderPitch
        transform =
            _shoulderYawTransform
            * Matrix4x4.TRS(
                _shoulderPitchOffset,
                Quaternion.AngleAxis(_shoulderPitchAngle + delta, _shoulderPitchAxis),
                Vector3.one
            )
            * _elbowPitchTransform
            * _wristPitchTransform
            * _wristYawTransform
            * _wristRollTransform;
        decompose(transform, pertubed);
        jacobian.SetColumn(1, (pertubed - origin) / delta);

        // ElbowPitch
        transform =
            _shoulderYawTransform
            * _shoulderPitchTransform
            * Matrix4x4.TRS(
                _elbowPitchOffset,
                Quaternion.AngleAxis(_elbowPitchAngle + delta, _elbowPitchAxis),
                Vector3.one
            )
            * _wristPitchTransform
            * _wristYawTransform
            * _wristRollTransform;
        decompose(transform, pertubed);
        jacobian.SetColumn(2, (pertubed - origin) / delta);

        // WristPitch
        transform =
            _shoulderYawTransform
            * _shoulderPitchTransform
            * _elbowPitchTransform
            * Matrix4x4.TRS(
                _wristPitchOffset,
                Quaternion.AngleAxis(_wristPitchAngle + delta, _wristPitchAxis),
                Vector3.one
            )
            * _wristYawTransform
            * _wristRollTransform;
        decompose(transform, pertubed);
        jacobian.SetColumn(3, (pertubed - origin) / delta);

        // WristYaw
        transform =
            _shoulderYawTransform
            * _shoulderPitchTransform
            * _elbowPitchTransform
            * _wristPitchTransform
            * Matrix4x4.TRS(
                _wristYawOffset,
                Quaternion.AngleAxis(_wristYawAngle + delta, _wristYawAxis),
                Vector3.one
            )
            * _wristRollTransform;
        decompose(transform, pertubed);
        jacobian.SetColumn(4, (pertubed - origin) / delta);

        // WristRoll
        transform =
            _shoulderYawTransform
            * _shoulderPitchTransform
            * _elbowPitchTransform
            * _wristPitchTransform
            * _wristYawTransform
            * Matrix4x4.TRS(
                _wristRollOffset,
                Quaternion.AngleAxis(_wristRollAngle + delta, _wristRollAxis),
                Vector3.one
            );
        decompose(transform, pertubed);
        jacobian.SetColumn(5, (pertubed - origin) / delta);
    }


    public void DrawDebugAxes(Matrix4x4 worldToBase)
    {
        var location = worldToBase;
        // Shoulder Yaw
        location = location * _shoulderYawTransform;
        DrawDebugAxis(location);
        // Shoulder Pitch
        location = location * _shoulderPitchTransform;
        DrawDebugAxis(location);
        // Elbow Pitch
        location = location * _elbowPitchTransform;
        DrawDebugAxis(location);
        // Wrist Pitch
        location = location * _wristPitchTransform;
        DrawDebugAxis(location);
        // Wrist Yaw
        location = location * _wristYawTransform;
        DrawDebugAxis(location);
        // Wrist Roll
        location = location * _wristRollTransform;
        DrawDebugAxis(location);
        // End effector
        location = location * _endEffectorTransform;
        DrawDebugAxis(location);
    }

    private void DrawDebugAxis(Matrix4x4 transform)
    {
        var origin = transform.MultiplyPoint3x4(Vector3.zero);
        Debug.DrawLine(
            origin,
            transform.MultiplyPoint3x4(Vector3.right),
            Color.red,
            0,
            false
        );
        Debug.DrawLine(
            origin,
            transform.MultiplyPoint3x4(Vector3.up),
            Color.green,
            0,
            false
        );
        Debug.DrawLine(
            origin,
            transform.MultiplyPoint3x4(Vector3.forward),
            Color.blue,
            0,
            false
        );
    }

    private void RecalculateTransforms()
    {
        _shoulderYawTransform = Matrix4x4.TRS(
            _shoulderYawOffset,
            Quaternion.AngleAxis(_shoulderYawAngle, _shoulderYawAxis),
            Vector3.one
        );
        _shoulderPitchTransform = Matrix4x4.TRS(
            _shoulderPitchOffset,
            Quaternion.AngleAxis(_shoulderPitchAngle, _shoulderPitchAxis),
            Vector3.one
        );
        _elbowPitchTransform = Matrix4x4.TRS(
            _elbowPitchOffset,
            Quaternion.AngleAxis(_elbowPitchAngle, _elbowPitchAxis),
            Vector3.one
        );
        _wristPitchTransform = Matrix4x4.TRS(
            _wristPitchOffset,
            Quaternion.AngleAxis(_wristPitchAngle, _wristPitchAxis),
            Vector3.one
        );
        _wristYawTransform = Matrix4x4.TRS(
            _wristYawOffset,
            Quaternion.AngleAxis(_wristYawAngle, _wristYawAxis),
            Vector3.one
        );
        _wristRollTransform = Matrix4x4.TRS(
            _wristRollOffset,
            Quaternion.AngleAxis(_wristRollAngle, _wristRollAxis),
            Vector3.one
        );

        ArmTransform =
            _shoulderYawTransform
            * _shoulderPitchTransform
            * _elbowPitchTransform
            * _wristPitchTransform
            * _wristYawTransform
            * _wristRollTransform;
    }
}

