using System;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof (CarController))]
    public class CarUserControl : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use


        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
        }

        private void Update(){
            float steerInput = 0;
            if (Input.GetKey (KeyCode.LeftArrow))
                steerInput = -1;
            if (Input.GetKey (KeyCode.RightArrow))
                steerInput = 1;

            float accel = 0;
            if (Input.GetKey (KeyCode.UpArrow))
                accel = 1;
            float footbrake = 0 ;
            if (Input.GetKey (KeyCode.DownArrow))
                footbrake = 1;

            float handbrake = 0;

            if (Input.GetKey (KeyCode.X))
                handbrake = 1;

            //bool accelKey = Input.GetKey (KeyCode.UpArrow);
		    //bool brakeKey = Input.GetKey (KeyCode.DownArrow);

            m_Car.Move(steerInput, accel, footbrake, handbrake); //Move(float steering, float accel, float footbrake, float handbrake)
        }

/*
        private void FixedUpdate()
        {
            // pass the input to the car!
            
            float h = CrossPlatformInputManager.GetAxis("Horizontal");
            float v = CrossPlatformInputManager.GetAxis("Vertical");
#if !MOBILE_INPUT
            float handbrake = CrossPlatformInputManager.GetAxis("Jump");
            m_Car.Move(h, v, v, handbrake);
#else
            m_Car.Move(h, v, v, 0f);
#endif

        }
        */
    }
}
