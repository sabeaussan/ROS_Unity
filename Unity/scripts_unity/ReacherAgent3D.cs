using UnityEngine;
using MLAgents;
using MLAgents.Sensors;

public class ReacherAgent3D : Agent
{
    // Game Object
    public GameObject segment1;
    public GameObject segment2;
    public GameObject hand;
    public GameObject arm_base;
    public GameObject goal;


    // Variable d'entrainement
    public float goal_speed;        // Vitesse de déplacement de la balle
    public int dev_freq;            // Fréquence à laquelle la balle change de direction (en step de simu)
    public float positive_range;    // distance hand-goal à partir de laquelle la récompense est >0
    public float alpha;             // constante pour régler la reward positive


    
    private  float SEGMENT1_LENGTH;         // longueur du premier segment du bras
    private Vector3 ORIGIN;                 // position absolue de la base
    private static float ARM_RANGE = 4.5f;  // Longueur total du bras + hand


    // Angle pour la position du goal            
    private float m_Goal_phi;
    private float m_Goal_theta;
    // rayon de la sphère sur lequel se trouve le goal
    private float m_Goal_rho;

    // Signe pour le déplacement du goal
    private int sign_x;
    private int sign_y;
    private int sign_z;

    private float episode_reward;
    private int nb_step;
    



    public override void Initialize()
    {
        episode_reward = 0f;
        SEGMENT1_LENGTH = segment2.transform.position.y;
        ORIGIN = arm_base.transform.position;

    }




    
    public override void OnEpisodeBegin()
    {

       nb_step = 0;
        // Place la sphère à atteindre à une position 
        // Calcul fait en coordonnées sphériques
        
        do{
            sign_x = Random.Range(-1,2);
            sign_y = Random.Range(-1,2);
            sign_z = Random.Range(-1,2);
        }while(sign_x == 0 && sign_y == 0 && sign_z==0);

        episode_reward = 0f;
        SpawnGoal();
        

    }

    public void UpdateGoalPosition(){
    	float MIN_RANGE = 1.5f;
        nb_step++;
        if(nb_step%dev_freq ==0){
            // Change la direction de la balle fréquemment
            sign_x = Random.Range(-1,2);
            sign_y = Random.Range(-1,2);
            sign_z = Random.Range(-1,2);
        }

        // On check si bouger le goal ne le fait pas sortir des limites 
        Vector3 new_pos_x = goal.transform.localPosition + new Vector3(goal_speed * sign_x,0f,0f);
        Vector3 new_pos_y = goal.transform.localPosition + new Vector3(0f,goal_speed*sign_y,0f);
        Vector3 new_pos_z = goal.transform.localPosition + new Vector3(0f,0f,goal_speed*sign_z);


        float distance_x = Vector3.Distance(new_pos_x,arm_base.transform.localPosition);
        float distance_y = Vector3.Distance(new_pos_y,arm_base.transform.localPosition);
        float distance_z = Vector3.Distance(new_pos_z,arm_base.transform.localPosition);

        // Condition limite
        if( distance_x > ARM_RANGE - 0.1f || distance_x < MIN_RANGE ){
            sign_x = -sign_x;   
        }
        if( distance_z > ARM_RANGE - 0.1f || distance_z < MIN_RANGE ){
            sign_z = -sign_z;   
        }
        if( distance_y > ARM_RANGE - 0.1f || arm_base.transform.localPosition.y -0.25f > new_pos_y.y || distance_y < MIN_RANGE  ){
            sign_y = -sign_y;   
        }
        Vector3 increment = new Vector3(goal_speed * sign_x,goal_speed*sign_y,goal_speed*sign_z); 
        goal.transform.localPosition = increment + goal.transform.localPosition;

    }

    public void SpawnGoal(){
        m_Goal_theta = Random.Range(0, Mathf.PI/ 2f);
        m_Goal_phi = Random.Range(0, Mathf.PI);
        m_Goal_rho = Random.Range(2,ARM_RANGE);

        
        // Sphérique
        var goalX = m_Goal_rho * Mathf.Sin(m_Goal_theta) * Mathf.Cos(m_Goal_phi);
        var goalZ = m_Goal_rho * Mathf.Sin(m_Goal_theta) * Mathf.Sin(m_Goal_phi) ;
        var goalY = m_Goal_rho * Mathf.Cos(m_Goal_theta);


        goal.transform.localPosition = new Vector3(goalX, goalY, goalZ);
    }

    // We collect the normalized rotations, angularal velocities, and velocities of both
    // limbs of the reacher as well as the relative position of the target and hand.
    public override void CollectObservations(VectorSensor sensor)
    {
       
   
        sensor.AddObservation(segment1.transform.rotation); 			//4    
        sensor.AddObservation(segment2.transform.rotation);				//4
        sensor.AddObservation(arm_base.transform.rotation);				//4

        Vector3 goal_pos = goal.transform.position - ORIGIN;    // Ramène les coordonnées au centre de la scene
        Vector3 hand_pos = hand.transform.position - ORIGIN;    // Ramène les coordonnées au centre de la scene
        sensor.AddObservation(goal_pos);					//3
        sensor.AddObservation(hand_pos);        			//3

    }

    // The agent's four actions correspond to torques on each of the two joints.
    public override void OnActionReceived(float[] vectorAction)
    {
        float reward_obtained;

        // Rotation de la base 
        float consigne_base = Mathf.Clamp(vectorAction[2], -1f, 1f) * 180;
        arm_base.transform.rotation = Quaternion.Euler(0f,consigne_base,0f);
        

        // Rotation du segment 1
        float consigne_1 = Mathf.Clamp(vectorAction[0], -1f, 1f) * 90;
        segment1.transform.rotation =  Quaternion.Euler(0f,consigne_base,consigne_1);
        
        // Rotation du segment 2
        float consigne_2 = Mathf.Clamp(vectorAction[1], -1f, 1f) * 155 + consigne_1;
        segment2.transform.rotation =  Quaternion.Euler(0f,consigne_base,consigne_2);
        consigne_1 = consigne_1 * Mathf.PI/180f;

        // Reposition du segment 2
        consigne_base = consigne_base * Mathf.PI/180f;
        float pos_x = - SEGMENT1_LENGTH * Mathf.Sin(consigne_1);
        float pos_y = SEGMENT1_LENGTH * Mathf.Cos(consigne_1);
        float pos_z = pos_x * Mathf.Sin(consigne_base);

        pos_x = pos_x * Mathf.Cos(consigne_base);
        segment2.transform.localPosition = new Vector3(pos_x,pos_y + 0.5f,- pos_z);

        UpdateGoalPosition();

        float distance_to_target = Vector3.Distance(hand.transform.position,goal.transform.position);
        
        if(distance_to_target < positive_range){
            reward_obtained = 1/(distance_to_target * alpha);
            Debug.Log("Bingo !");
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
        action[2] = Input.GetAxis("Rotation");

        return action;
    }

    

    
}
