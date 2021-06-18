using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot_Controller : MonoBehaviour
{
    private int numJoints = 0;
    private ArticulationBody[] Arms;
    private List<ArticulationBody> Joints = new List<ArticulationBody>();
    public ArticulationBody robotBase;
    public float maxVel = 10f;
    public List<float> moveRobotTo = new List<float>(){100, -60, 120, 180, -90, 45};
    //private float maxVel = maxVelocity * Mathf.PI / 180f;
    // private Robot theRobot;
    public bool setAllJointDriveValues = true;
    public List<float> stiffness = new List<float>() {10000f, 10000f, 10000f, 10000f, 5000f, 5000f};
    public List<float> damping = new List<float>() {2000f, 2500f, 3000f, 1000f, 1000f, 1000f};
    // public List<float> forceLimit = new List<float>() {10f, 10f, 10f, 1.5f, 10f, 100f};
    private float tempMass;
    
    // Start is called before the first frame update
    void Start()
    {
        Arms = GetComponentsInChildren<ArticulationBody>(); //Robot must be in tree structure
        Debug.Log("We just found " + Arms.Length + " ArticulationBodies in this scene");
        foreach(ArticulationBody meow in Arms)
        {
            updateMaxForce(meow);
            /*
            if(setAllJointDriveValues)
            {
                ArticulationDrive drive = meow.xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                meow.xDrive = drive;
            }
            */
            if(meow.jointType == ArticulationJointType.PrismaticJoint || meow.jointType == ArticulationJointType.RevoluteJoint || meow.jointType == ArticulationJointType.SphericalJoint)
            {
                Joints.Add(meow);
                numJoints = numJoints + 1;
            }
        }
        Debug.Log("We just added " + numJoints + " ArticulationBodies to the joints array");

        if(stiffness.Count != numJoints || damping.Count != numJoints)
        {
            Debug.Log("Expected " + numJoints + " inputs and Got " + stiffness.Count + " and " + damping.Count + " for the stiffness and damping lists.");
        }

        if(setAllJointDriveValues)
        {
            for(int i=0; i<numJoints ; i++)
            {
                ArticulationDrive drive = Joints[i].xDrive;
                drive.stiffness = stiffness[i];
                drive.damping = damping[i];
                // drive.forceLimit = forceLimit[i];
                Joints[i].xDrive = drive;
            }
        }
    }

    void FixedUpdate()
    {
        if(Input.GetKeyDown(KeyCode.R) || Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0))
        {
            List<float> angles = new List<float>() {0, 0, 0, 0, 0, 0};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.A))
        {
            List<float> angles = new List<float>(){100, -60, 120, 180, -90, 45};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.S))
        {
            List<float> angles = new List<float>(){-45, -30, 15, 0, 90, 90};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.M))
        {
            adjustRobotAngles(moveRobotTo);
        }
        if(Input.GetKeyDown(KeyCode.Keypad1) || Input.GetKeyDown(KeyCode.Alpha1))
        {
            List<float> angles = new List<float>() {90, 0, 0, 0, 0, 0};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.Keypad2) || Input.GetKeyDown(KeyCode.Alpha2))
        {
            List<float> angles = new List<float>() {0, -90, 0, 0, 0, 0};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.Keypad3) || Input.GetKeyDown(KeyCode.Alpha3))
        {
            List<float> angles = new List<float>() {0, 0, -90, 0, 0, 0};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.Keypad4) || Input.GetKeyDown(KeyCode.Alpha4))
        {
            List<float> angles = new List<float>() {0, 0, 0, 90, 0, 0};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.Keypad5) || Input.GetKeyDown(KeyCode.Alpha5))
        {
            List<float> angles = new List<float>() {0, 0, 0, 0, -90, 0};
            adjustRobotAngles(angles);
        }
        if(Input.GetKeyDown(KeyCode.Keypad6) || Input.GetKeyDown(KeyCode.Alpha6))
        {
            List<float> angles = new List<float>() {0, 0, 0, 0, 0, 90};
            adjustRobotAngles(angles);
        }
    }
    
    public void adjustRobotAngles(List<float> angles)
    {
        if(angles.Count != numJoints)
        {
            Debug.Log("Incorrect number of input angles. Expected " + numJoints + " but received " + angles.Count + ".");
            return;
        }
        Debug.Log("We are going to change the angle positions to "+ angles);
        for(int i = 0; i < angles.Count ; i++)
        {
            ArticulationBody joint = Joints[i];
            //updateMaxForce(joint);
            float r = radiusOfRotation(joint);
            joint.maxAngularVelocity = maxVel / r;
            ArticulationDrive drive = joint.xDrive;
            drive.target = angles[i];
            joint.xDrive = drive;
            Vector3 worldAngVel = joint.angularVelocity;
            Vector3 angVel = transform.InverseTransformVector(worldAngVel);
            if(angVel.magnitude > maxVel)
            {
                Debug.Log("The max velocity of " + maxVel +" has been exceeded to " + angVel.magnitude);
            }
            else
            {
                Debug.Log("The velocity has remained under " + maxVel);
            }
            r = radiusOfRotation(joint);
            joint.maxAngularVelocity = maxVel / r;
        }
    }

    public Vector3 findCenterOfMassFromJoint(ArticulationBody joint)
    {
        //List<float> masses = new List<float>() {joint.mass};
        //List<Vector3> COMs = new List<Vector3>() {joint.worldCenterOfMass};
        ArticulationBody[] children = joint.GetComponentsInChildren<ArticulationBody>();
        float totalMass = 0;
        Vector3 com = Vector3.zero;
        foreach(ArticulationBody child in children)
        {
            //masses.Add(child.mass);
            totalMass = totalMass + child.mass;
            //COMs.Add(child.worldCenterOfMass);
            Vector3 weightedCOM = child.worldCenterOfMass * child.mass;
            com = com + weightedCOM;
        }
        tempMass = totalMass;
        return com / totalMass;
    }

    public float radiusOfRotation(ArticulationBody joint)
    {
        Vector3 com = findCenterOfMassFromJoint(joint);
        Vector3 pos = transform.TransformPoint(joint.anchorPosition);
        float radius = Vector3.Distance(com, pos);
        return radius;
    }

    public void updateMaxForce(ArticulationBody joint)
    {
        float radius = radiusOfRotation(joint);
        ArticulationDrive drive = joint.xDrive;
        drive.forceLimit = tempMass * maxVel * maxVel * radius;
        joint.xDrive = drive;
    }
}
