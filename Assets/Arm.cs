using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using TMPro;
using UnityEngine;

public class Arm : MonoBehaviour
{
    private Controls _controls;
    private Motors _motors;
    private Kinematics _kinematics;

    private Vector<float> _jointAngles;
    private Matrix<float> _jacobian;
    private Vector<float> _state_a;
    private Vector<float> _state_b;

    public GameObject ShoulderYawJoint;
    public GameObject ShoulderPitchJoint;
    public GameObject ElbowPitchJoint;
    public GameObject WristPitchJoint;
    public GameObject WristYawJoint;
    public GameObject WristRollJoint;

    public GameObject Hubble;

    public TextMeshProUGUI ModeText;
    public GameObject HelpPanel;

    void Start()
    {
        _controls = new Controls();
        _motors = new Motors(
            ShoulderYawJoint.transform,
            ShoulderPitchJoint.transform,
            ElbowPitchJoint.transform,
            WristPitchJoint.transform,
            WristYawJoint.transform,
            WristRollJoint.transform
        );
        _kinematics = new Kinematics(
            Vector3.up, new Vector3(0, 0, 0),
            Vector3.right, new Vector3(0, 0.467f, 0),
            Vector3.right, new Vector3(0, 0, 6.943f),
            Vector3.right, new Vector3(0, 0, 7.63f),
            Vector3.up, new Vector3(0, 0, 1.174f),
            Vector3.forward, new Vector3(0, 0, 1.196f),
            new Vector3(0, 0, 0.546f)
        );

        _jointAngles = DenseVector.Create(6, 0);
        _jacobian = DenseMatrix.Create(12, 6, 0);
        _state_a = DenseVector.Create(12, 0);
        _state_b = DenseVector.Create(12, 0);

        HelpPanel.SetActive(false);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.H))
        {
            HelpPanel.SetActive(!HelpPanel.activeSelf);
        }

        if (HelpPanel.activeSelf)
        {
            return;
        }

        if (Input.GetKey(KeyCode.R))
        {
            _motors.Reset();
            return;
        }

        if (Input.GetKey(KeyCode.Tab))
        {
            Hubble.transform.SetParent(WristRollJoint.transform);
            //Hubble.transform.SetParent(this);
        }


        _motors.Update();

        _motors.GetAngles(_jointAngles);
        _kinematics.SetAngles(_jointAngles);

        _kinematics.DrawDebugAxes(transform.worldToLocalMatrix);

        _controls.Update();
        ModeText.text = _controls.StatusText;

        if (_controls.SelectedMode == Controls.Mode.Direct)
        {
            switch (_controls.SelectedJoint)
            {
                case Controls.Joint.ShoulderYaw:
                    _motors.SetVelocities(shoulderYaw: _controls.InputDirect);
                    break;
                case Controls.Joint.ShoulderPitch:
                    _motors.SetVelocities(shoulderPitch: _controls.InputDirect);
                    break;
                case Controls.Joint.ElbowPitch:
                    _motors.SetVelocities(elbowPitch: _controls.InputDirect);
                    break;
                case Controls.Joint.WristPitch:
                    _motors.SetVelocities(wristPitch: _controls.InputDirect);
                    break;
                case Controls.Joint.WristYaw:
                    _motors.SetVelocities(wristYaw: _controls.InputDirect);
                    break;
                case Controls.Joint.WristRoll:
                    _motors.SetVelocities(wristRoll: _controls.InputDirect);
                    break;
            }
        }

        if (_controls.SelectedMode == Controls.Mode.Controlled)
        {
            // TODO: Move POR out from end-effector?
            var desiredTransform = _kinematics.ArmTransform;

            switch (_controls.SelectedPOR)
            {
                case Controls.PointOfReference.EndEffector:
                    desiredTransform = _kinematics.ArmTransform * Matrix4x4.TRS(_controls.InputTranslation, Quaternion.Euler(_controls.InputRotation), Vector3.one);
                    break;

                case Controls.PointOfReference.Ship:
                    var rotation = Quaternion.Euler(_controls.InputRotation) * _kinematics.ArmTransform.rotation;
                    var translation = _controls.InputTranslation + (Vector3)_kinematics.ArmTransform.GetColumn(3);
                    desiredTransform = Matrix4x4.TRS(translation, rotation, Vector3.one);
                    break;
            }

            // Get desired change in state-space
            _kinematics.FiniteDiffJacobian(_jacobian, _state_a, _state_b, TransformToVector, delta: 1);

            TransformToVector(_kinematics.ArmTransform, _state_a);
            TransformToVector(desiredTransform, _state_b);

            var diffState = _state_b - _state_a;
            var controlVelocities = SolveForVelocities(_jacobian, diffState);

            _motors.SetVelocities(controlVelocities);

        }
    }

    private Vector<float> SolveForVelocities(Matrix<float> jacobian, Vector<float> dstate)
    {
        // TODO: There are multiple ways to implement/tune this
        var vel = jacobian.Solve(dstate);

        return vel;
    }

    private static void TransformToVector(Matrix4x4 transform, Vector<float> column)
    {
        column[0] = transform.m00;
        column[1] = transform.m01;
        column[2] = transform.m02;
        column[3] = transform.m03;
        column[4] = transform.m10;
        column[5] = transform.m11;
        column[6] = transform.m12;
        column[7] = transform.m13;
        column[8] = transform.m20;
        column[9] = transform.m21;
        column[10] = transform.m22;
        column[11] = transform.m23;
    }
}
