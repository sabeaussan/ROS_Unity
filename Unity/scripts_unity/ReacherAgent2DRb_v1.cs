using UnityEngine;
using MLAgents;
using MLAgents.Sensors;

// TODO : convertir consigne en angle pour l'observation

public class ReacherAgent : Agent
{
    public GameObject segment1;
    public GameObject segment2;
    public GameObject hand;
    public GameObject arm_base;
    public GameObject joint;
    public GameObject goal;
    public float goal_speed;
    public int dev_freq;
    public float positive_range;

    
    private  float SEGMENT1_LENGTH;
    private Vector3 ORIGIN;
    private static float ARM_RANGE = 4.4f;

    private float m_Goal_theta;
    private float m_Goal_phi;
    private float m_Goal_rho;

    private float episode_reward;
    private int sign_x;
    private int sign_y;
    private int sign_z;
    private int nb_step;
    
    private Rigidbody rb1;
    private Rigidbody rb2;
    private Rigidbody rbb;
    private ConfigurableJoint config_joint;

    // Collect the rigidbodies of the reacher in order to resue them for
    // observations and actions.
    public override void Initialize()
    {
        episode_reward = 0f;
        SEGMENT1_LENGTH = segment2.transform.position.y;
        ORIGIN = arm_base.transform.position;
        rb1 = segment1.GetComponent<Rigidbody>();
        rb2 = segment2.GetComponent<Rigidbody>();
        rbb = arm_base.GetComponent<Rigidbody>();
        config_joint = segment1.GetComponent<ConfigurableJoint>();
    }




    // Resets the position  of the agent and the goal.
    public override void OnEpisodeBegin()
    {

       nb_step = 0;
        // Place la sphère à atteindre à une position 
        // Calcul fait en coordonnées sphériques
        
        do{
            sign_x = Random.Range(-1,2);
            sign_y = Random.Range(-1,2);
            //sign_z = Random.Range(-1,2);
        }while(sign_x == 0 && sign_y == 0 /*&& sign_z==0*/);

        episode_reward = 0f;
        SpawnGoal();
        

    }

    public void UpdateGoalPosition(){
        float MIN_RANGE = 1.5f;
        nb_step++;
        if(nb_step%dev_freq ==0){
            sign_x = Random.Range(-1,2);
            sign_y = Random.Range(-1,2);
            //sign_z = Random.Range(-1,2);
        }
        Vector3 new_pos_x = goal.transform.localPosition + new Vector3(goal_speed * sign_x,0f,0f);
        Vector3 new_pos_y = goal.transform.localPosition + new Vector3(0f,goal_speed*sign_y,0f);
        //Vector3 new_pos_z = goal.transform.localPosition + new Vector3(0f,0f,goal_speed*sign_z);


        float distance_x = Vector3.Distance(new_pos_x,arm_base.transform.localPosition);
        float distance_y = Vector3.Distance(new_pos_y,arm_base.transform.localPosition);
        //float distance_z = Vector3.Distance(new_pos_z,arm_base.transform.localPosition);

        // Condition limite
        if( distance_x > ARM_RANGE - 0.1f || distance_x < MIN_RANGE ){
            sign_x = -sign_x;   
        }
        /*if( distance_z > ARM_RANGE - 0.1f || distance_z < MIN_RANGE ){
            sign_z = -sign_z;   
        }*/
        if( distance_y > ARM_RANGE - 0.1f || arm_base.transform.localPosition.y -0.25f > new_pos_y.y || distance_y < MIN_RANGE  ){
            sign_y = -sign_y;   
        }
        Vector3 increment = new Vector3(goal_speed * sign_x,goal_speed*sign_y,0f); 
        goal.transform.localPosition = increment + goal.transform.localPosition;

    }

    public void SpawnGoal(){
        m_Goal_theta = Random.Range(0, Mathf.PI/ 2f);
        m_Goal_phi = Random.Range(0, Mathf.PI);
        m_Goal_rho = Random.Range(2,ARM_RANGE);

        
        // Sphérique
        var goalX = m_Goal_rho * Mathf.Sin(m_Goal_theta) /** Mathf.Cos(m_Goal_phi)*/;
        //var goalZ = m_Goal_rho * Mathf.Sin(m_Goal_theta) * Mathf.Sin(m_Goal_phi) ;
        var goalY = m_Goal_rho * Mathf.Cos(m_Goal_theta);


        goal.transform.localPosition = new Vector3(goalX, goalY, 0f);
    }

    // We collect the normalized rotations, angularal velocities, and velocities of both
    // limbs of the reacher as well as the relative position of the target and hand.
    public override void CollectObservations(VectorSensor sensor)
    {
        var angle_S1 =  segment1.transform.rotation.eulerAngles.z;
        var angle_S2 = segment2.transform.rotation.eulerAngles.z;
        if(segment1.transform.rotation.eulerAngles.z > 120f){
            angle_S1 = segment1.transform.rotation.eulerAngles.z - 360f;
        }
        if(segment2.transform.rotation.eulerAngles.z > 190f){
            angle_S2 = segment2.transform.rotation.eulerAngles.z - 360f;
            angle_S2 = angle_S2 - angle_S1;
        }
        
        //Debug.Log("angle_S1 : "+angle_S1);
        //Debug.Log("angularVelocity 1 : "+rb1.angularVelocity);
        //Debug.Log("Velocity 1 : "+rb1.velocity);
        sensor.AddObservation(angle_S1);                                    //1
        sensor.AddObservation(rb1.angularVelocity);                         //3
        sensor.AddObservation(rb1.velocity);                                //3

        //Debug.Log("angle_S2 : "+angle_S2);
        //Debug.Log("angularVelocity 2 : "+rb2.angularVelocity);
        //Debug.Log("Velocity 2 : "+rb2.velocity);
        sensor.AddObservation(angle_S2);                                    //1
        sensor.AddObservation(rb2.angularVelocity);                         //3
        sensor.AddObservation(rb2.velocity);                                //3


        //sensor.AddObservation(arm_base.transform.rotation.eulerAngles.y);   //1
        //sensor.AddObservation(rbb.angularVelocity.y);                       //1

        Vector3 goal_pos = goal.transform.position - ORIGIN;
        Vector3 hand_pos = hand.transform.position - ORIGIN;
        sensor.AddObservation(goal_pos);                    //3
        sensor.AddObservation(hand_pos);                    //3
        //sensor.AddObservation(goal_speed);                  //1

        // dim_obs = 23

    }

    // The agent's four actions correspond to torques on each of the two joints.
    public override void OnActionReceived(float[] vectorAction)
    {
        float reward_obtained;
        //config_joint.angularZMotion = ConfigurableJointMotion.Free;

        
        // Rotation segment 2
        var torque = Mathf.Clamp(vectorAction[1], -1f, 1f) * 10f;
        Debug.Log("torque s2: "+ vectorAction[1]);
        if(torque == 0){
            //rb2.constraints |= RigidbodyConstraints.FreezeRotationZ;
            //config_joint.angularZMotion = ConfigurableJointMotion.Locked;
        }
        else {
            //rb2.constraints &= ~ RigidbodyConstraints.FreezeRotationZ;
            //config_joint.angularZMotion = ConfigurableJointMotion.Free;
        }
        rb2.AddRelativeTorque(new Vector3(0f, 0f, torque));


        // Rotation segment 1
        torque = Mathf.Clamp(vectorAction[0], -1f, 1f) * 10f;
        Debug.Log("torque s1: "+ vectorAction[0]);
        if(torque == 0){
            //rb1.constraints |= RigidbodyConstraints.FreezeRotationZ;
        }
        else {
            //rb1.constraints &= ~ RigidbodyConstraints.FreezeRotationZ;
        }
        rb1.AddRelativeTorque(new Vector3(0f, 0f, torque));


        // Rotation base
        /*torque = Mathf.Clamp(vectorAction[2], -1f, 1f) * 10f;
        Debug.Log("torque base: "+ torque);
        if(torque == 0){
            rbb.constraints |= RigidbodyConstraints.FreezeRotationY;
        }
        else {
            rbb.constraints &= ~ RigidbodyConstraints.FreezeRotationY;
        }
        rbb.AddRelativeTorque(new Vector3(0f,torque,0f));*/

        


        


        UpdateGoalPosition();

        float distance_to_target = Vector3.Distance(hand.transform.position,goal.transform.position);
        
        if(distance_to_target < positive_range){
            reward_obtained = 0.1f;
        }
        else {
            reward_obtained = -1.0f  * distance_to_target;

        }
        SetReward(reward_obtained);
        
    }



    public override float[] Heuristic()
    {
        var action = new float[3];
        action[0] = Input.GetAxis("Horizontal");
        action[1] = Input.GetAxis("Vertical");
        //action[2] = Input.GetAxis("Rotation");

        return action;
    }

    //public addFixedJoint(GameObject obj){
    //    obj.
    //}

    
}
