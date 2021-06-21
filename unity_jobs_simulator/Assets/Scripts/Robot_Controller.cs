using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot_Controller : MonoBehaviour
{
    private ArticulationBody[] _Arms; // This array tracks all the articulation bodies in the robot
    private List<ArticulationBody> _Joints = new List<ArticulationBody>(); // This list tracks all the articulation bodies with moveable joints in the robot
    public ArticulationBody robotBase; // This public variable is assigned to the base Articulation body in the robot
    public float maxVelocity = 90f; // This is the velocity which the robot joint cannot exceed in deg/s 
    private float _maxVel; // This is a conversion of maxVelocity to rad/s
    public List<float> targetPos; // This is the input variable that we tell the robot to move to (upon pressing the key 'M')
    private List<float> _currentDrivePos; // This is a track of the robot's Drive target
    public List<float> _enroute; // This is a track of the final target to which the robot is trying to get to i.e. where the Drive will eventually reach
    private List<float> _incAngles; // This is a list of the incremental angles with which the joints will increase by for each iteration
    private List<float> _nextAngles; // This is the next angle, within the increments, that the robot Drive will go to i.e. the sum of the increments
    private List<int> _increments; // This is a list of integers of the number of increments that each joint must move through
    private List<float> _zeroList; // This is a simple reference list of all zeroes; its size is _Joints.Count
    public float angleSensitivity; // This is the error which the angles can be, measured in deg (Should be 0.088) todo tune masses of each articulation body
    public bool setAllJointDriveValues = true; // This will override the preset values of the stiffness and damping for the joints and set it to the inputs
    public List<float> stiffness; // This is the input for the stiffnesses for each joint to override the preset
    public List<float> damping; // This is the input for the dampings for each joint to override the preset
    private float _pi = Mathf.PI; // Easier vaiable reference for the value of pi
    private TimeSpan _t1; // The initial time for any movement
    private TimeSpan _t2; // The final time for when a movement is complete
    private DateTime _startTime;
    // Matrices for each of the Transposes:
    public Matrix4x4 Real_T_Unity;
    public Matrix4x4 UnityRobotBase_T_UnityRobotTCP;
    public Matrix4x4 RealRobotBase_T_RealRobotTCP;

    // temp variables for debugging below
    public List<float> currentPos;
    
    void Start()
    {
        _startTime = DateTime.Now;
        initializeJoints(); // This function initializes the _Arms/_Joints array/list for the robot
        
        _zeroList = new List<float>(); // An easy reference list of zeroes
        for(int i=0 ; i < _Joints.Count ; i++)
        {
            _zeroList.Add(0);
        }

        targetPos = new List<float>(_zeroList); // Simple initial position
        _currentDrivePos = new List<float>(_zeroList); // This is the current position at the Start
        _incAngles = new List<float>(_zeroList); // The incremental angles are all zero
        _enroute = new List<float>(_zeroList); // We aren't enroute anywhere
        stiffness = new List<float>() {10000f, 100000000000000000000000000000000000000f, 1000000f, 1000000f, 5000f, 5000f};
        damping = new List<float>() {2000f, 25000f, 3000f, 1000f, 1000f, 1000f};

        _maxVel = maxVelocity / 180 * _pi; // This converts our degree/second velocity to radians/second

        Real_T_Unity = Matrix4x4.identity; // Creating the Matrix
        Real_T_Unity[0, 0] = -1; // The Matrix represents a reflection in y-z plane... doesn't need an update
        updateRealRobotBase_T_RealRobotTCP(); // This updates both of the other Matrices
    }

    public void initializeJoints()
    {
        _Arms = GetComponentsInChildren<ArticulationBody>(); // This takes all articulation bodies connected to the base and orders it from the top of the tree
        Debug.Log("We just found " + _Arms.Length + " ArticulationBodies in this scene");
        foreach(ArticulationBody meow in _Arms) // Now we run through each articulation body and add it to the List _Joints for those that aren't fixed
        {
            if(meow.jointType == ArticulationJointType.PrismaticJoint || meow.jointType == ArticulationJointType.RevoluteJoint || meow.jointType == ArticulationJointType.SphericalJoint)
            {
                _Joints.Add(meow); // This is where we add the joint 'meow' since we know it isn't a fixed joint and can move 
            }
        }
        Debug.Log("We just added " + _Joints.Count + " ArticulationBodies to the joints array");

        if(stiffness.Count != _Joints.Count || damping.Count != _Joints.Count) // Error message if we don't have correct number of inputs
        {
            Debug.Log("Expected " + _Joints.Count + " inputs and Got " + stiffness.Count + " and " + damping.Count + " for the stiffness and damping lists.");
        }

        if(setAllJointDriveValues) // We set the Stiffness and Damping values to override the preset if told to do so
        {
            for(int i=0; i < _Joints.Count ; i++) // Runs through all the joints to set each value to the xDrive for each joint
            {
                ArticulationDrive drive = _Joints[i].xDrive;
                drive.stiffness = stiffness[i];
                drive.damping = damping[i];
                _Joints[i].xDrive = drive;
            }
        }
    }

    void Update() //This only updates the target position variable depending on the input we receive for that frame
    {
        if(Input.GetKeyDown(KeyCode.R) || Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0))
        {
            targetPos = new List<float>() {0, 0, 0, 0, 0, 0};
        }
        if(Input.GetKeyDown(KeyCode.A))
        {
            targetPos = new List<float>(){100, -60, 120, 180, -90, 45};
        }
        if(Input.GetKeyDown(KeyCode.S))
        {
            targetPos = new List<float>(){-45, -30, 15, 0, 90, 90};
        }
        if(Input.GetKeyDown(KeyCode.Keypad1) || Input.GetKeyDown(KeyCode.Alpha1))
        {
            targetPos = new List<float>() {90, 0, 0, 0, 0, 0};
        }
        if(Input.GetKeyDown(KeyCode.Keypad2) || Input.GetKeyDown(KeyCode.Alpha2))
        {
            targetPos = new List<float>() {0, -90, 0, 0, 0, 0};
        }
        if(Input.GetKeyDown(KeyCode.Keypad3) || Input.GetKeyDown(KeyCode.Alpha3))
        {
            targetPos = new List<float>() {0, 0, -90, 0, 0, 0};
        }
        if(Input.GetKeyDown(KeyCode.Keypad4) || Input.GetKeyDown(KeyCode.Alpha4))
        {
            targetPos = new List<float>() {0, 0, 0, 90, 0, 0};
        }
        if(Input.GetKeyDown(KeyCode.Keypad5) || Input.GetKeyDown(KeyCode.Alpha5))
        {
            targetPos = new List<float>() {0, 0, 0, 0, -90, 0};
        }
        if(Input.GetKeyDown(KeyCode.Keypad6) || Input.GetKeyDown(KeyCode.Alpha6))
        {
            targetPos = new List<float>() {0, 0, 0, 0, 0, 90};
        }
        if(Input.GetKeyDown(KeyCode.T))
        {
            Vector3 pos = getTCPTranspose();
            Debug.Log("The robot tool is at position " + pos.ToString());
        }
    }

    void FixedUpdate()
    {
        updateCurrentDrivePos(); // We need to update the current position of the Robot to ensure we know exactly where it is
        
        if(samePos(_currentDrivePos, targetPos)) // If we are at the preset target, FixedUpdate shouldn't run so we return nothing
        {
            // checkAngleErrors();
            updateRealRobotBase_T_RealRobotTCP();
            return;
        }

        if(!(samePos(_currentDrivePos, _enroute))) // If the route has changed, we need to restart the motion
        {
            _enroute = new List<float>(targetPos);
            _incAngles = new List<float>(_zeroList);
        }

        if(samePos(_incAngles, _zeroList)) // If we don't have increments for the angles, then we aren't going anywhere and need to initialize these increments
        {
            getIncrements(); // This calculates the values for increments given the target angles, the current positions, and the maximum velocity
            setIncAngles(); // This calculates the incremental angles given the increments and the target angles
            _enroute = new List<float>(targetPos); // We set our enroute position as this is where our robot movement will be moving to
            _nextAngles = addLists(_currentDrivePos, _incAngles); // This increments the angles by one increment
            _t1 = _startTime.Subtract(DateTime.Now); // Time.realtimeSinceStartup; // We record the start time for this motion
        }

        if(samePos(_nextAngles, targetPos)) // If the next Angle is the final target position, then this is the final increment
        {
            adjustRobotAngles(targetPos, 0f); // We set the final angle to the final target, and we set the joint the velocities to zero
            _incAngles = new List<float>(_zeroList); // We've reached the final location so we do not want to increment the angles anymore so we set this to zero
            _t2 = _startTime.Subtract(DateTime.Now); // Time.realtimeSinceStartup; // We record the final time 
            TimeSpan timeDiff = _t2.Subtract(_t1);
            Debug.Log("Time took for that motion was: " + timeDiff.ToString()); // Log the time taken
        }
        else // Otherwise, the robot is incrementing the angles to the next incremental position
        {
            adjustRobotAngles(_nextAngles, _maxVel); // We adjust the robot to the next position
            _nextAngles = addLists(_nextAngles, _incAngles); // We increment the nextAngle list up by the incremental angles
        }
    }

    public void updateCurrentDrivePos()
    {
        _currentDrivePos = new List<float>(); // Creates a new list to store the current Drive target positions in
        for(int i = 0; i < _Joints.Count ; i++) // Runs through each Joint to check its position
        {
            ArticulationBody joint = _Joints[i];
            float temp = joint.xDrive.target;
            _currentDrivePos.Add(temp);
        }

        //This is temporary for debugging reasons
        currentPos = new List<float>(); 
        for(int i = 0; i < _Joints.Count ; i++) 
        {
            ArticulationBody joint = _Joints[i];
            float tempp = joint.jointPosition[0];
            currentPos.Add(tempp * 180f / _pi);
        }
    }

    public void getIncrements()
    {
        _increments = new List<int>();
        for(int i=0 ; i < targetPos.Count ; i++)
        {
            float diff = Math.Abs(targetPos[i] - _currentDrivePos[i]); // We get the magnitude of the angle needed to move
            float t = diff/_maxVel; // The time taken is the angle divided by the angular velocity
            int newT = (int)t; // Each increment is dependent on the time step
            if(newT == 0) // If the time step is zero, this means that there is no split for the angle so we simlpy have an increment of 1
            {
                _increments.Add(1);
            }
            else // Otherwise, we just add the incremental value
            {
                _increments.Add(newT); 
            }
        }
    }

    public void setIncAngles()
    {
        _incAngles = new List<float>();
        for(int i=0 ; i<_increments.Count ; i++) // Each incremental angle is the total angle needed to travel divided by the number of increments
        {
            _incAngles.Add((targetPos[i] - _currentDrivePos[i])/_increments[i]);
        }
    }

    public List<float> addLists(List<float> list1, List<float> list2) // This is a simple list element-wise adder
    {
        if(list1.Count != list2.Count)
        {
            Debug.Log("You cannot add a list of size " + list1.Count + " to a list of size " + list2.Count);
        }
        List<float> temp = new List<float>();
        for(int i=0 ; i < list1.Count ; i++)
        {
            float tempp = list1[i] + list2[i];
            temp.Add(tempp);
        }
        return temp;
    }

    public bool samePos(List<float> pos1, List<float> pos2) // This function runs through each element in the lists and check to see if they're within range of each other
    {
        if(pos1.Count != pos2.Count)
        {
            return false;
        }
        for(int i = 0; i < pos1.Count; i++)
        {
            if(Math.Abs(pos1[i] - pos2[i]) > angleSensitivity) // if any angle isn't within 'angleSensitivity' range of each other, then it is false
            {
                return false;
            }
        }
        return true; // After checking all, if none returns false, then we are within range and we return true
    }

    public void adjustRobotAngles(List<float> angles, float vel) // This simply adjusts the robot angles to the values in the angles input list and sets the velocity to vel
    {
        if(angles.Count != _Joints.Count)
        {
            Debug.Log("Incorrect number of input angles. Expected " + _Joints.Count + " but received " + angles.Count + ".");
            return;
        }

        for(int i = 0; i < angles.Count; i++) // We run through each joint and set the Drive targets and velocities
        {
            ArticulationBody joint = _Joints[i];
            ArticulationDrive drive = joint.xDrive;
            float angle = angles[i];
            drive.target = angle;
            drive.targetVelocity = vel;
            joint.xDrive = drive;
        }
    }

    public void checkAngleErrors()
    {
        for(int i = 0; i < _Joints.Count ; i++) // Runs through each Joint to check its position
        {
            ArticulationBody joint = _Joints[i];
            float radAng = joint.jointPosition[0]; // This gives us the angle position in radians 
            float ang = radAng / _pi * 180f; // We convert back to degrees to check since sensitivity is given in degrees
            if(Math.Abs(ang - _currentDrivePos[i]) > angleSensitivity)
            {
                Debug.Log("Joint " + joint.ToString() + " exceeds the Angle error of " + angleSensitivity + ". It is at " + ang + " and should be at " + _currentDrivePos[i] + ".");
            }
        }
    }

    public Vector3 getTCPTranspose()
    {
        int final = _Arms.Length - 1;
        ArticulationBody last = _Arms[final];
        Vector3 worldTranspose = last.transform.position;
        // Vector3 worldTranspose = Transform.TransformVector(localTranspose);
        return worldTranspose;
    }

    public void updateUnityRobotBase_T_UnityRobotTCP()
    {
        int final = _Arms.Length - 1;
        ArticulationBody last = _Arms[final]; // Pull the tool which is the final Articulation Body in _Arms
        UnityRobotBase_T_UnityRobotTCP = last.transform.localToWorldMatrix; // Unity has built in function for the Matrix
    }

    public void updateRealRobotBase_T_RealRobotTCP()
    {
        updateUnityRobotBase_T_UnityRobotTCP(); // We ensure to update the Unity Robot transform
        Matrix4x4 Unity_T_Real = Real_T_Unity.inverse; // Get the inverse 
        RealRobotBase_T_RealRobotTCP = Real_T_Unity * UnityRobotBase_T_UnityRobotTCP * Unity_T_Real; // Simple Transform relationship
    }
}
