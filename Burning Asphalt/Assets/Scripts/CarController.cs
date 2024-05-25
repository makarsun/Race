using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class CarController : MonoBehaviour
{
    [System.Serializable]
    public struct wheelinfo
    {
        public Transform wheel;
        public WheelCollider rb_wheel;
    }
    [SerializeField]
    private float steer = 30f, brake = 300f;

    [SerializeField]
    private float motor0 = -100f, motor0_5k = 100f,
                  motor1k = 400f, motor1_5k = 500f,
                  motor2k = 650f, motor2_5k = 700f,
                  motor3k = 725f, motor3_5k = 700f,
                  motor4k = 675f, motor4_5k = 650f,
                  motor5k = 640f, motor5_5k = 630f,
                  motor6k = 620f, motor6_5k = 420f,
                  motor7k = 250f, motor7_5k = 0f,
                  motor8k = -150f, engineRPM = 600f, TMT, PMT, minWheelRPM = 100f;

    [SerializeField]
    private List<float> Gearbox = new List<float>();

    [SerializeField]
    private wheelinfo FL, RL, RR, FR;

    [SerializeField]
    private WheelCollider engine;

    private float throttle_axis = 0f, brake_axis = 0f, steer_axis = 0f, handbrake_axis = 0f, clutch_axis = 0f;
    private int gear = 0, last_gear = 1;

    private bool EngineIsWork, LightIsWork;

    private Vector3 wheel_position;
    private Quaternion wheel_rotation;

    private Multitool _input;

    private void Awake()
    {
        _input = new Multitool();
        _input.InputButtons.Throttle.performed += context => OnThrottle();
        _input.InputButtons.Brake.performed += context => OnBrake();
        _input.InputButtons.Steering.performed += context => OnSteering();
        _input.InputButtons.Handbrake.performed += context => OnHandbrake();
        /*_input.InputButtons.Engine.performed += context => OnStartStop();
        _input.InputButtons.Light.performed += context => OnLight();*/
        _input.InputButtons.Gearbox.performed += context => OnForwardBackward();
        _input.InputButtons.Neutral.performed += context => OnNeutral();
    }
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.V))
        {
            EngineIsWork = !EngineIsWork;
        }
        if (Input.GetKeyDown(KeyCode.L))
        {
            LightIsWork = !LightIsWork;
        }
    }
    private void FixedUpdate()
    {
        SetMotorTorque();
        PracticTorque();
        SeeGear();
        UpdateVisualWheels();
        SimulateTorque();
    }
    private void OnThrottle()
    {
        if (EngineIsWork)
        {
            throttle_axis = _input.InputButtons.Throttle.ReadValue<float>();
        }
    }
    private void OnBrake()
    {
        brake_axis = _input.InputButtons.Brake.ReadValue<float>();
    }
    private void OnSteering()
    {
        steer_axis = _input.InputButtons.Steering.ReadValue<float>();
    }
    private void OnHandbrake()
    {
        handbrake_axis = _input.InputButtons.Handbrake.ReadValue<float>();
    }
    /*private void OnStartStop()
    {
        EngineIsWork = _input.InputButtons.Engine.ReadValue<bool>();
    }
    private void OnLight()
    {
        LightIsWork = _input.InputButtons.Light.ReadValue<bool>();
    }*/
    private void OnForwardBackward()
    {
        switch (gear)
        {
            case -1:
                CheckGear(1);
                break;
            case 0:
                CheckGear(last_gear);
                break;
            case 1:
                CheckGear(-1);
                break;
        }
    }
    private void OnNeutral()
    {
        if (gear != 0)
        {
            CheckGear(0);
        }
        else
        {
            CheckGear(last_gear);
        }
    }
    private void SetMotorTorque()
    {
        if (engineRPM < motor0)
        {
            TMT = motor0 + engineRPM;
        }
        else if (engineRPM < motor0_5k)
        {
            TMT = motor0 + ((motor0_5k - motor0) * (engineRPM - motor0) / 500f);
        }
        else if (engineRPM < motor1k)
        {
            TMT = motor0_5k + ((motor1k - motor0_5k) * (engineRPM - motor0_5k) / 500f);
        }
        else if (engineRPM < motor1_5k)
        {
            TMT = motor1k + ((motor1_5k - motor1k) * (engineRPM - motor1k) / 500f);
        }
        else if (engineRPM < motor2k)
        {
            TMT = motor1_5k + ((motor2k - motor1_5k) * (engineRPM - motor1_5k) / 500f);
        }
        else if (engineRPM < motor2_5k)
        {
            TMT = motor2k + ((motor2_5k - motor2k) * (engineRPM - motor2k));
        }
        else if (engineRPM < motor3k)
        {
            TMT = motor2_5k + ((motor3k - motor2_5k) * (engineRPM - motor2_5k) / 500f);
        }
        else if (engineRPM < motor3_5k)
        {
            TMT = motor3k + ((motor3_5k - motor3k) * (engineRPM - motor3k) / 500f);
        }
        else if (engineRPM < motor4k)
        {
            TMT = motor3_5k + ((motor4k - motor3_5k) * (engineRPM - motor3_5k) / 500f);
        }
        else if (engineRPM < motor4_5k)
        {
            TMT = motor4k + ((motor4_5k - motor4k) * (engineRPM - motor4k) / 500f);
        }
        else if (engineRPM < motor5k)
        {
            TMT = motor4_5k + ((motor5k - motor4_5k) * (engineRPM - motor4_5k) / 500f);
        }
        else if (engineRPM < motor5_5k)
        {
            TMT = motor5k + ((motor5_5k - motor5k) * (engineRPM - motor5k) / 500f);
        }
        else if (engineRPM < motor6k)
        {
            TMT = motor5_5k + ((motor6k - motor5_5k) * (engineRPM - motor5_5k) / 500f);
        }
        else if (engineRPM < motor6_5k)
        {
            TMT = motor6k + ((motor6_5k - motor6k) * (engineRPM - motor6k) / 500f);
        }
        else if (engineRPM < motor7k)
        {
            TMT = motor6_5k + ((motor7k - motor6_5k) * (engineRPM - motor6_5k) / 500f);
        }
        else if (engineRPM < motor7_5k)
        {
            TMT = motor7k + ((motor7_5k - motor7k) * (engineRPM - motor7k) / 500f);
        }
        else if (engineRPM < motor8k)
        {
            TMT = motor7_5k + ((motor8k - motor7_5k) * (engineRPM - motor7_5k) / 500f);
        }
        else
        {
            TMT = motor8k - (engineRPM / 500f);
        }
        TMT *= 7f;
    }
    private void PracticTorque()
    {
        if (TMT > 0f)
        {
            if (throttle_axis <= 0.2f)
            {
                PMT = TMT * 0.4f;
            }
            else
            {
                PMT = TMT * throttle_axis;
            }
        }
        else
        {
            PMT = TMT;
        }
    }
    private void SeeGear()
    {
        if (gear != 0)
        {
            last_gear = gear;
        }
        if (gear >= 1 && engineRPM >= 6000f)
        {
            CheckGear(gear + 1);
        }
        if (gear >= 1 && engineRPM <= 2000f)
        {
            CheckGear(gear - 1);
        }
    }
    private void CheckGear(int planned)
    {
        for (float i = 0; i < 1f; i += 0.01f)
        {
            clutch_axis = i;
        }
        gear = planned;
        if (RL.rb_wheel.rpm <= minWheelRPM || RR.rb_wheel.rpm <= minWheelRPM)
        {
            for (float i = 1; i > 0.5f; i -= 0.01f)
            {
                clutch_axis = i;
            }
        }
        clutch_axis = 0;
    }
    private void UpdateVisualWheels()
    {
        FL.rb_wheel.steerAngle = steer * steer_axis;
        FR.rb_wheel.steerAngle = steer * steer_axis;

        FL.rb_wheel.GetWorldPose(out wheel_position, out wheel_rotation);
        FL.wheel.position = wheel_position;
        FL.wheel.rotation = wheel_rotation;

        FR.rb_wheel.GetWorldPose(out wheel_position, out wheel_rotation);
        FR.wheel.position = wheel_position;
        FR.wheel.rotation = wheel_rotation;

        RL.rb_wheel.GetWorldPose(out wheel_position, out wheel_rotation);
        RL.wheel.position = wheel_position;
        RL.wheel.rotation = wheel_rotation;

        RR.rb_wheel.GetWorldPose(out wheel_position, out wheel_rotation);
        RR.wheel.position = wheel_position;
        RR.wheel.rotation = wheel_rotation;
    }
    private void SimulateTorque()
    {
        engine.motorTorque = PMT;
        engine.brakeTorque = (engine.rpm + ((RL.rb_wheel.rpm + RR.rb_wheel.rpm) / 2)) * (1 - clutch_axis);
        if (gear == -1)
        {
            RL.rb_wheel.motorTorque = PMT * clutch_axis / Gearbox[0];
            RR.rb_wheel.motorTorque = PMT * clutch_axis / Gearbox[0];
        }
        if (gear >= 1)
        {
            RL.rb_wheel.motorTorque = PMT * clutch_axis / Gearbox[gear];
            RR.rb_wheel.motorTorque = PMT * clutch_axis / Gearbox[gear];
        }
        FL.rb_wheel.brakeTorque = brake * brake_axis;
        FR.rb_wheel.brakeTorque = brake * brake_axis;
        if (handbrake_axis > 0.2f)
        {
            RL.rb_wheel.brakeTorque = brake * brake_axis;
            RR.rb_wheel.brakeTorque = brake * brake_axis;
        }
        else
        {
            RL.rb_wheel.brakeTorque = brake * handbrake_axis;
            RR.rb_wheel.brakeTorque = brake * handbrake_axis;
        }
        Debug.Log("engine:" + engine.rpm);
        Debug.Log("TMT:" + TMT);
        Debug.Log("PMT:" + PMT);
        /*Debug.Log("FL:" + FL.rb_wheel.rpm);
        Debug.Log("FR:" + FR.rb_wheel.rpm);
        Debug.Log("RL:" + RL.rb_wheel.rpm);
        Debug.Log("RR:" + RR.rb_wheel.rpm);*/
    }
    private void OnEnable()
    {
        _input.Enable();
    }
    private void OnDisable()
    {
        _input.Disable();
    }
}