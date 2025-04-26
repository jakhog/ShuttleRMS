using System;
using UnityEngine;

public class Controls
{
    private readonly float _directSpeed = 30f;
    private readonly float _translateSpeed = 1.0f;
    private readonly float _rotateSpeed = 10.0f;

	public Controls()
	{
		SelectedMode = Mode.Direct;
		SelectedJoint = Joint.ShoulderYaw;
        SelectedPOR = PointOfReference.Ship;

        InputDirect = 0;
        InputTranslation = Vector3.zero;
        InputRotation = Vector3.zero;
    }

	public Mode SelectedMode { get; private set; }

	public Joint SelectedJoint { get; private set; }

    public PointOfReference SelectedPOR { get; private set; }



    public float InputDirect { get; private set; }

    public Vector3 InputTranslation { get; private set; }

    public Vector3 InputRotation { get; private set; }



	public void Update()
	{
        ReadMode();
        ReadInputs();
    }


    public string StatusText
    {
        get => (SelectedMode, SelectedJoint, SelectedPOR) switch
        {
            (Mode.Direct, Joint.ShoulderYaw, _) => "1 - Direct: Shoulder Yaw ",
            (Mode.Direct, Joint.ShoulderPitch, _) => "2 - Direct: Shoulder Pitch ",
            (Mode.Direct, Joint.ElbowPitch, _) => "3 - Direct: Elbow Pitch ",
            (Mode.Direct, Joint.WristPitch, _) => "4 - Direct: Wrist Pitch ",
            (Mode.Direct, Joint.WristYaw, _) => "5 - Direct: Wrist Yaw ",
            (Mode.Direct, Joint.WristRoll, _) => "6 - Direct: Wrist Roll ",
            (Mode.Controlled, _, PointOfReference.Ship) => "7 - Controlled: Ship ",
            (Mode.Controlled, _, PointOfReference.EndEffector) => "8 - Controlled: End Effector ",
            _ => "Invalid mode!?! "
        } + InputText;
    }

    public string InputText
    {
        get => SelectedMode switch
        {
            Mode.Direct => $"({InputDirect:F2})",
            Mode.Controlled => $"({InputTranslation.x:F2},{InputTranslation.y:F2},{InputTranslation.z:F2}) [{InputRotation.x:F2},{InputRotation.y:F2},{InputRotation.z:F2}]",
            _ => "(Invalid mode!?!)"
        };
    }


	public enum Mode
	{
		Direct,
		Controlled,
	}

	public enum Joint
	{
		None,
		ShoulderYaw,
		ShoulderPitch,
		ElbowPitch,
		WristPitch,
		WristYaw,
		WristRoll,
	}

	public enum PointOfReference
	{
		Ship,
		EndEffector,
    }


    private void ReadMode()
    {
        if (Input.GetKey(KeyCode.Alpha1))
        {
            SelectedMode = Mode.Direct;
            SelectedJoint = Joint.ShoulderYaw;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha2))
        {
            SelectedMode = Mode.Direct;
            SelectedJoint = Joint.ShoulderPitch;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha3))
        {
            SelectedMode = Mode.Direct;
            SelectedJoint = Joint.ElbowPitch;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha4))
        {
            SelectedMode = Mode.Direct;
            SelectedJoint = Joint.WristPitch;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha5))
        {
            SelectedMode = Mode.Direct;
            SelectedJoint = Joint.WristYaw;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha6))
        {
            SelectedMode = Mode.Direct;
            SelectedJoint = Joint.WristRoll;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha7))
        {
            SelectedMode = Mode.Controlled;
            SelectedJoint = Joint.None;
            SelectedPOR = PointOfReference.Ship;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha8))
        {
            SelectedMode = Mode.Controlled;
            SelectedJoint = Joint.None;
            SelectedPOR = PointOfReference.EndEffector;
            return;
        }
        if (Input.GetKey(KeyCode.Alpha9))
        {
            return;
        }
    }

    private void ReadInputs()
    {
        float direct = 0;
        float x = 0, y = 0, z = 0;

        if (Input.GetKey(KeyCode.I))
        {
            z = _translateSpeed;
            direct = _directSpeed;
        }
        else if (Input.GetKey(KeyCode.K))
        {
            z = -_translateSpeed;
            direct = -_directSpeed;
        }

        if (Input.GetKey(KeyCode.L))
        {
            x = _translateSpeed;
        }
        else if (Input.GetKey(KeyCode.J))
        {
            x = -_translateSpeed;
        }

        if (Input.GetKey(KeyCode.U))
        {
            y = _translateSpeed;
        }
        else if (Input.GetKey(KeyCode.O))
        {
            y = -_translateSpeed;
        }

        if (Input.GetKey(KeyCode.UpArrow))
        {
            direct = _directSpeed;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            direct = -_directSpeed;
        }

        InputDirect = direct;
        InputTranslation = new Vector3(x, y, z);

        x = 0; y = 0; z = 0;

        if (Input.GetKey(KeyCode.W))
        {
            x = _rotateSpeed;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            x = -_rotateSpeed;
        }

        if (Input.GetKey(KeyCode.A))
        {
            z = _rotateSpeed;
        }
        else if (Input.GetKey(KeyCode.D))
        {
            z = -_rotateSpeed;
        }

        if (Input.GetKey(KeyCode.E))
        {
            y = _rotateSpeed;
        }
        else if (Input.GetKey(KeyCode.Q))
        {
            y = -_rotateSpeed;
        }

        InputRotation = new Vector3(x, y, z);
    }
}

