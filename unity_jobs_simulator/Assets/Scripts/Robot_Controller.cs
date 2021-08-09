using System;
using System.Collections.Generic;
using UnityEngine;

public class Robot_Controller : MonoBehaviour
{
    private ArticulationBody[] _links; // This array tracks all the articulation bodies in the robot
    private List<ArticulationBody> _joints = new List<ArticulationBody>(); // This list tracks all the articulation bodies with moveable joints in the robot
    public ArticulationBody RobotBase; // This public variable is assigned to the base Articulation body in the robot
    public float MaxVelocityDeg = 90f; // This is the velocity which the robot joint cannot exceed in deg/s 
    private float _maxVelRad; // This is a conversion of MaxVelocityDeg to rad/s
    public float AngleSensitivityDeg; // This is the error which the angles can be, measured in deg (Should be 0.088) todo tune masses of each articulation body
    public List<float> TargetJointAngles; // This is the input variable that we tell the robot to move to
    public List<float> CurrentJointAngles; // This is a track of the robot's Joint Angles in Degrees
    private List<float> _currentDriveTarget; // This is a track of the robot's Drive target
    private List<float> _currentEndGoal; // This is a track of the final target to which the robot is trying to get to i.e. where the Drive will eventually reach
    private List<float> _incAngles; // This is a list of the incremental angles with which the joints will increase by for each iteration
    private List<float> _nextAngles; // This is the next angle, within the increments, that the robot Drive will go to i.e. the sum of the increments
    private List<float> _increments; // This is a list of integers of the number of increments that each joint must move through
    private List<float> _zeroList; // This is a simple reference list of all zeroes; its size is _joints.Count
    public bool CheckAngleError; // This is a variable to toggle that would post the angles which are exceeding the angle sensitivity
    public bool SetAllJointDriveValues = true; // This will override the preset values of the stiffness and damping for the joints and set it to the inputs
    public List<float> Stiffness; // This is the input for the stiffnesses for each joint to override the preset
    public List<float> Damping; // This is the input for the dampings for each joint to override the preset
    
    void Start()
    {
        InitializeJoints(); // This function initializes the _links/_joints array/list for the robot
        
        _zeroList = new List<float>(); // An easy reference list of zeroes
        for(int i=0 ; i < _joints.Count ; i++)
        {
            _zeroList.Add(0);
        }

        if(TargetJointAngles.Count != _joints.Count) // If an initial position hasn't been set, we set the joint angls to all zeroes
        {
            TargetJointAngles = new List<float>(_zeroList); // Simple initial position
        }
        _currentDriveTarget = new List<float>(_zeroList); // This is the current position at the Start
        _incAngles = new List<float>(_zeroList); // The incremental angles are all zero
        _nextAngles = new List<float>(_zeroList);
        _increments = new List<float>(_zeroList);
        _currentEndGoal = new List<float>(_zeroList); // We aren't enroute anywhere
        _maxVelRad = MaxVelocityDeg * Mathf.Deg2Rad; // This converts our degree/second velocity to radians/second
    }

    public void InitializeJoints()
    {
        _links = RobotBase.GetComponentsInChildren<ArticulationBody>(); // This takes all articulation bodies connected to the base and orders it from the top of the tree
        Debug.Log("We just found " + _links.Length + " ArticulationBodies in this scene");
        foreach(ArticulationBody meow in _links) // Now we run through each articulation body and add it to the List _joints for those that aren't fixed
        {
            if(meow.jointType == ArticulationJointType.RevoluteJoint)
            {
                _joints.Add(meow); // This is where we add the joint 'meow' since we know it isn't a fixed joint and can move 
            }
            else if(meow.jointType == ArticulationJointType.PrismaticJoint || meow.jointType == ArticulationJointType.SphericalJoint)
            {
                Debug.LogError("All the joints must be of type Fixed or Revolute.", meow);
            }
        }
        Debug.Log("We just added " + _joints.Count + " ArticulationBodies to the joints array");

        if(Stiffness.Count != _joints.Count || Damping.Count != _joints.Count) // Error message if we don't have correct number of inputs
        {
            Debug.LogError("Expected " + _joints.Count + " inputs and Got " + Stiffness.Count + " and " + Damping.Count + " for the stiffness and damping lists.");
        }

        if(SetAllJointDriveValues) // We set the Stiffness and Damping values to override the preset if told to do so
        {
            for(int i=0; i < _joints.Count ; i++) // Runs through all the joints to set each value to the xDrive for each joint
            {
                ArticulationDrive drive = _joints[i].xDrive;
                drive.stiffness = Stiffness[i];
                drive.damping = Damping[i];
                _joints[i].xDrive = drive;
            }
        }
    }

    void Update() //This only updates the target position variable depending on the input we receive for that frame
    {
        if(Input.GetKeyDown(KeyCode.R) || Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0))
        {
            TargetJointAngles.Clear();
            TargetJointAngles.AddRange(_zeroList);
        }
        if(Input.GetKeyDown(KeyCode.A))
        {
            List<float> testPos1 = new List<float>(){100, -60, 120, 180, -90, 45};
            TargetJointAngles.Clear();
            TargetJointAngles.AddRange(testPos1);
        }
        if(Input.GetKeyDown(KeyCode.S))
        {
            List<float> testPos2 = new List<float>(){-45, -30, 15, 0, 90, 90};
            TargetJointAngles.Clear();
            TargetJointAngles.AddRange(testPos2);
        }
    }

    void FixedUpdate()
    {
        UpdateCurrentDriveTargetAndJointAngles(); // We need to update the current position of the Robot to ensure we know exactly where it is
        
        if(SamePos(CurrentJointAngles, TargetJointAngles)) // If we are at the preset target, FixedUpdate shouldn't run so we return nothing
        {
            return;
        }

        if(!(SamePos(_currentDriveTarget, _currentEndGoal))) // If the route has changed, we need to restart the motion
        {
            _currentEndGoal.Clear();
            _currentEndGoal.AddRange(TargetJointAngles);
            _incAngles.Clear();
            _incAngles.AddRange(_zeroList);
        }

        // This if condition initializes the motion
        if(SamePos(_incAngles, _zeroList)) // If we don't have increments for the angles, then we aren't going anywhere and need to initialize these increments
        {
            _maxVelRad = MaxVelocityDeg * Mathf.Deg2Rad; // This updates the value of _maxVelRad if the input MaxVelocityDeg is changed during motion
            GetAndSetIncrements(); // This calculates the values for incremental angles given the target angles, the current positions, and the maximum velocity
            _currentEndGoal.Clear();
            _currentEndGoal.AddRange(TargetJointAngles); // We set our the end goal position of our robot
            _nextAngles = AddLists(_currentDriveTarget, _incAngles); // This increments the angles by one increment
        }

        if(SamePos(_nextAngles, TargetJointAngles)) // If the next Angle is the final target position, then this is the final increment
        {
            AdjustRobotAngles(TargetJointAngles, 0f); // We set the final angle to the final target, and we set the joint the velocities to zero
            _incAngles.Clear();
            _incAngles.AddRange(_zeroList); // We've reached the final location so we do not want to increment the angles anymore so we set this to zero
        }
        else // Otherwise, the robot is incrementing the angles to the next incremental position
        {
            AdjustRobotAngles(_nextAngles, _maxVelRad); // We adjust the robot to the next position
            _nextAngles = AddLists(_nextAngles, _incAngles); // We increment the nextAngle list up by the incremental angles
        }
    }

    public void UpdateCurrentDriveTargetAndJointAngles()
    {
        _currentDriveTarget.Clear(); // Clears the list to store the current Drive target positions in
        CurrentJointAngles.Clear();
        for(int i = 0; i < _joints.Count ; i++) // Runs through each Joint to check its position
        {
            ArticulationBody joint = _joints[i];
            float temp1 = joint.xDrive.target;
            _currentDriveTarget.Add(temp1);
            float temp2 = joint.jointPosition[0];
            CurrentJointAngles.Add(temp2 * Mathf.Rad2Deg);
        }
    }

    public void GetAndSetIncrements()
    {
        _increments.Clear();
        for(int i=0 ; i < TargetJointAngles.Count ; i++)
        {
            float diff = Math.Abs(TargetJointAngles[i] - _currentDriveTarget[i]); // We get the magnitude of the angle needed to move
            float t = diff/_maxVelRad; // The time taken is the angle divided by the angular velocity
            int newT = (int)t; // Each increment is dependent on the time step
            if (newT == 0) // If the time step is zero, this means that there is no split for the angle so we simlpy have an increment of 1
                _increments.Add(1);
            else // Otherwise, we just add the incremental value
                _increments.Add(newT); 
        }
        _incAngles.Clear();
        for(int i=0 ; i<_increments.Count ; i++) // Each incremental angle is the total angle needed to travel divided by the number of increments
        {
            _incAngles.Add((TargetJointAngles[i] - _currentDriveTarget[i])/_increments[i]);
        }
    }

    public List<float> AddLists(List<float> list1, List<float> list2) // This is a simple list element-wise adder
    {
        if(list1.Count != list2.Count)
        {
            Debug.LogError("You cannot add a list of size " + list1.Count + " to a list of size " + list2.Count);
        }
        List<float> temp = new List<float>();
        for(int i=0 ; i < list1.Count ; i++)
        {
            float tempp = list1[i] + list2[i];
            temp.Add(tempp);
        }
        return temp;
    }

    public bool SamePos(List<float> pos1, List<float> pos2) // This function runs through each element in the lists and check to see if they're within range of each other
    {
        if(pos1.Count != pos2.Count)
        {
            return false;
        }
        for(int i = 0; i < pos1.Count; i++)
        {
            if(Math.Abs(pos1[i] - pos2[i]) > AngleSensitivityDeg) // if any angle isn't within 'AngleSensitivityDeg' range of each other, then it is false
            {
                return false;
            }
        }
        return true; // After checking all, if none returns false, then we are within range and we return true
    }

    public void AdjustRobotAngles(List<float> angles, float vel) // This simply adjusts the robot angles to the values in the angles input list and sets the velocity to vel
    {
        if(angles.Count != _joints.Count)
        {
            Debug.LogError("Incorrect number of input angles. Expected " + _joints.Count + " but received " + angles.Count + ".");
            return;
        }

        for(int i = 0; i < angles.Count; i++) // We run through each joint and set the Drive targets and velocities
        {
            ArticulationBody joint = _joints[i];
            ArticulationDrive drive = joint.xDrive;
            float angle = angles[i];
            drive.target = angle;
            drive.targetVelocity = vel;
            joint.xDrive = drive;
        }
    }
}
