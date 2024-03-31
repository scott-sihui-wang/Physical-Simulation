//Name: Sihui Wang, Student ID: 301474102 (swa279)

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DiceRigidbody : MonoBehaviour
{
    [HideInInspector]
    public bool simPaused;

    public Vector3 initialPosition;
    public Quaternion initialRotation;
    public Vector3 initialVelocity;

    public float dt;
    public float mass;
    public float ks;
    public float kd;
    public float mu;

    private Vector3 position;
    private Quaternion rotation;
    private Vector3 linear_velocity;
    private Vector3 angular_velocity;

    private Vector3[] localVertices = {
        new Vector3(0.5f, 0.5f, 0.5f),
        new Vector3(0.5f, 0.5f, -0.5f),
        new Vector3(0.5f, -0.5f, 0.5f),
        new Vector3(0.5f, -0.5f, -0.5f),
        new Vector3(-0.5f, 0.5f, 0.5f),
        new Vector3(-0.5f, 0.5f, -0.5f),
        new Vector3(-0.5f, -0.5f, 0.5f),
        new Vector3(-0.5f, -0.5f, -0.5f)
    };

    private Matrix3x3 GetInertiaRefMatrix()
    {
        return new Matrix3x3(
            mass / 6f * Vector3.right,
            mass / 6f * Vector3.up,
            mass / 6f * Vector3.forward
        );
    }

    private List<Vector3> GetCollidedVertices()
    {
        List<Vector3> CollidedVertices = new List<Vector3>();

        /*** Part 1: Collision Detection ***/
        Matrix3x3 Rot=new Matrix3x3(rotation);
        for(int i=0; i<localVertices.Length; i++){
            if((position+Rot*localVertices[i]).y<0.0f){
                CollidedVertices.Add(localVertices[i]);
            }
        }
        /*** part 1 coding ends ***/

        return CollidedVertices;
    }

    private (Vector3, Vector3) ComputeForceAndTorque()
    {
        Vector3 netForce = Vector3.zero;
        Vector3 netTorque = Vector3.zero;

        /*** Part 2: Calculate Forces and Torques ***/
        netForce.y=-9.81f*mass;
        List<Vector3> CollidedVertices=GetCollidedVertices();
        Matrix3x3 Rot=new Matrix3x3(rotation);
        for(int i=0;i<CollidedVertices.Count;i++){
            Vector3 r=Rot*CollidedVertices[i];
            Vector3 position_vertex=position+r;
            Vector3 v_rel=Vector3.Cross(angular_velocity,r);
            Vector3 v_vertex=v_rel+linear_velocity;
            Vector3 f=Vector3.zero;
            float fn=-ks*position_vertex.y-kd*v_vertex.y;
            f.y=fn;
            float v_norm=Mathf.Sqrt(v_vertex.x*v_vertex.x+v_vertex.z*v_vertex.z);
            if(v_norm!=0){
                float ff=Mathf.Abs(mu*fn);
                f.x=-ff*v_vertex.x/v_norm;
                f.z=-ff*v_vertex.z/v_norm;
            }
            netForce+=f;
            netTorque+=Vector3.Cross(r,f);
        }
        /*** part 2 coding ends ***/

        return (netForce, netTorque);
    }

    private void Integrate(float deltaTime)
    {
        var (force, torque) = ComputeForceAndTorque();

        /*** Part 3: Integrate Timestep ***/
        Vector3 acceleration=force/mass;
        linear_velocity.x=linear_velocity.x+acceleration.x*deltaTime;
        linear_velocity.y=linear_velocity.y+acceleration.y*deltaTime;
        linear_velocity.z=linear_velocity.z+acceleration.z*deltaTime;
        position.x=position.x+linear_velocity.x*deltaTime;
        position.y=position.y+linear_velocity.y*deltaTime;
        position.z=position.z+linear_velocity.z*deltaTime;
        Matrix3x3 Rot=new Matrix3x3(rotation);
        Matrix3x3 I=Rot*GetInertiaRefMatrix()*Rot.transpose;
        angular_velocity=angular_velocity+I.inverse*(torque-Vector3.Cross(angular_velocity,I*angular_velocity))*deltaTime;
        Quaternion upd=new Quaternion();
        upd.w=0;
        upd.x=angular_velocity.x*deltaTime/2.0f;
        upd.y=angular_velocity.y*deltaTime/2.0f;
        upd.z=angular_velocity.z*deltaTime/2.0f;
        upd=upd*rotation;
        rotation.w+=upd.w;
        rotation.x+=upd.x;
        rotation.y+=upd.y;
        rotation.z+=upd.z;
        rotation=Quaternion.Normalize(rotation);
        /*** part 3 coding ends ***/
    }

    public void AdvanceTimeStep()
    {
        Integrate(dt);
    }

    public void ResetState()
    {
        position = initialPosition;
        linear_velocity = initialVelocity;
        rotation = initialRotation;
        angular_velocity = Vector3.zero;
}

    // Start is called before the first frame update
    void Start()
    {
        ResetState();
    }

    // FixedUpdate is called every fixed frame-rate frame
    // Read more here: https://docs.unity3d.com/ScriptReference/MonoBehaviour.FixedUpdate.html
    void FixedUpdate()
    {
        if (!simPaused)
        {
            AdvanceTimeStep();
        }
        transform.position = position;
        transform.rotation = rotation;
        Time.fixedDeltaTime = dt;
        //print(rotation);
        //Matrix3x3 Rot=new Matrix3x3(rotation);
        //print(rotation*localVertices[0]);
        //print(position);
        //print(position+rotation*localVertices[0]);
        //print((position+rotation*localVertices[0]).y);
    }
}
