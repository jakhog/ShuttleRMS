using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine;

public class Motors
{
    private readonly Motor _shoulderYaw;
    private readonly Motor _shoulderPitch;
    private readonly Motor _elbowPitch;
    private readonly Motor _wristPitch;
    private readonly Motor _wristYaw;
    private readonly Motor _wristRoll;

    private readonly Vector<float> _velocities;
    private readonly Vector<float> _scaling;

    public Motors(
        Transform shoulderYaw,
        Transform shoulderPitch,
        Transform elbowPitch,
        Transform wristPitch,
        Transform wristYaw,
        Transform wristRoll
    )
    {
        _shoulderYaw = new Motor(
            shoulderYaw,
            Vector3.up,
            -90, 90,
            10
        );
        _shoulderPitch = new Motor(
            shoulderPitch,
            Vector3.right,
            -180, 0,
            10
        );
        _elbowPitch = new Motor(
            elbowPitch,
            Vector3.right,
            0, 150,
            10
        );
        _wristPitch = new Motor(
            wristPitch,
            Vector3.right,
            -100, 100,
            20
        );
        _wristYaw = new Motor(
            wristYaw,
            Vector3.up,
            -90, 90,
            20
        );
        _wristRoll = new Motor(
            wristRoll,
            Vector3.forward,
            -180, 180,
            20
        );

        _velocities = DenseVector.Create(6, 0);
        _scaling = DenseVector.Create(6, 0);
    }

    public void SetVelocities(
        float shoulderYaw = 0,
        float shoulderPitch = 0,
        float elbowPitch = 0,
        float wristPitch = 0,
        float wristYaw = 0,
        float wristRoll = 0
    )
    {
        _velocities[0] = shoulderYaw;
        _velocities[1] = shoulderPitch;
        _velocities[2] = elbowPitch;
        _velocities[3] = wristPitch;
        _velocities[4] = wristYaw;
        _velocities[5] = wristRoll;
        ScaleAndSetVelocities();
    }

    public void SetVelocities(Vector<float> velocities)
    {
        _velocities.SetSubVector(0, velocities.Count, velocities);
        ScaleAndSetVelocities();
    }

    public void GetAngles(Vector<float> angles)
    {
        if (angles.Count > 0) angles[0] = _shoulderYaw.Angle;
        if (angles.Count > 1) angles[1] = _shoulderPitch.Angle;
        if (angles.Count > 2) angles[2] = _elbowPitch.Angle;
        if (angles.Count > 3) angles[3] = _wristPitch.Angle;
        if (angles.Count > 4) angles[4] = _wristYaw.Angle;
        if (angles.Count > 5) angles[5] = _wristRoll.Angle;
    }

    public void Update()
    {
        _shoulderYaw.Update();
        _shoulderPitch.Update();
        _elbowPitch.Update();
        _wristPitch.Update();
        _wristYaw.Update();
        _wristRoll.Update();
    }

    public void Reset()
    {
        _velocities[0] = 0;
        _velocities[1] = 0;
        _velocities[2] = 0;
        _velocities[3] = 0;
        _velocities[4] = 0;
        _velocities[5] = 0;
        _scaling[0] = 0;
        _scaling[1] = 0;
        _scaling[2] = 0;
        _scaling[3] = 0;
        _scaling[4] = 0;
        _scaling[5] = 0;

        _shoulderYaw.Reset();
        _shoulderPitch.Reset();
        _elbowPitch.Reset();
        _wristPitch.Reset();
        _wristYaw.Reset();
        _wristRoll.Reset();
    }

    private void ScaleAndSetVelocities()
    {
        _scaling[0] = _velocities[0] / _shoulderYaw.MaxVelocity;
        _scaling[1] = _velocities[1] / _shoulderPitch.MaxVelocity;
        _scaling[2] = _velocities[2] / _elbowPitch.MaxVelocity;
        _scaling[3] = _velocities[3] / _wristPitch.MaxVelocity;
        _scaling[4] = _velocities[4] / _wristYaw.MaxVelocity;
        _scaling[5] = _velocities[5] / _wristRoll.MaxVelocity;

        var maxFactor = _scaling.AbsoluteMaximum();
        if (maxFactor > 1)
        {
            _velocities[0] /= maxFactor;
            _velocities[1] /= maxFactor;
            _velocities[2] /= maxFactor;
            _velocities[3] /= maxFactor;
            _velocities[4] /= maxFactor;
            _velocities[5] /= maxFactor;
        }

        _shoulderYaw.Velocity = _velocities[0];
        _shoulderPitch.Velocity = _velocities[1];
        _elbowPitch.Velocity = _velocities[2];
        _wristPitch.Velocity = _velocities[3];
        _wristYaw.Velocity = _velocities[4];
        _wristRoll.Velocity = _velocities[5];
    }


    class Motor
    {
        private readonly Transform _transform;
        private readonly Vector3 _axis;
        private readonly float _limitLow;
        private readonly float _limitHigh;

        public Motor(Transform transform, Vector3 axis, float limitLow, float limitHigh, float maxVelocity)
        {
            _transform = transform;
            _axis = axis;
            _limitLow = limitLow;
            _limitHigh = limitHigh;

            _transform.localRotation = Quaternion.identity;
            Angle = 0;
            Velocity = 0;
            MaxVelocity = maxVelocity;
        }

        public float Angle { get; private set; }

        public float Velocity { get; set; }

        public float MaxVelocity { get; private set; }

        public bool HardLimited
        {
            get => Angle <= _limitLow || Angle >= _limitHigh;
        }

        public void Update()
        {
            Angle += Velocity * Time.deltaTime;

            if (Angle <= _limitLow)
            {
                Angle = _limitLow;
            }
            else if (Angle >= _limitHigh)
            {
                Angle = _limitHigh;
            }

            _transform.localRotation = Quaternion.AngleAxis(Angle, _axis);
        }

        public void Reset()
        {
            _transform.localRotation = Quaternion.identity;
            Angle = 0;
            Velocity = 0;
        }
    }
}

