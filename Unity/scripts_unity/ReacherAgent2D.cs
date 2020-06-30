using UnityEngine;
using MLAgents;
using MLAgents.Sensors;

public class ReacherAgent2D : Agent
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
    // rayon du cercle sur lequel se trouve le goal
    private float m_Goal_rho;


    // Signe pour le déplacement du goal
    private int sign_x;
    private int sign_y;

    private float episode_reward;
    private int nb_step;
    



    public override void Initialize()
    {
        episode_reward = 0f;
        SEGMENT1_LENGTH = segment2.transform.position.y;
        ORIGIN = arm_base.transform.position;

    }




    // Resets the position  of the agent and the goal.
    public override void OnEpisodeBegin()
    {

       nb_step = 0;

        // Place la sphère à atteindre à une position random        
        do{
            sign_x = Random.Range(-1,2);
            sign_y = Random.Range(-1,2);
        }while(sign_x == 0 && sign_y == 0);
        episode_reward = 0f;
        SpawnGoal();        
        

    }

    public void UpdateGoalPosition(){
        nb_step++;
        if(nb_step%dev_freq ==0){
            // Change la direction de la balle fréquemment
            sign_x = Random.Range(-1,2);
            sign_y = Random.Range(-1,2);
        }

        // On check si bouger le goal ne le fait pas sortir des limites 
        Vector3 new_pos_x = goal.transform.localPosition + new Vector3(goal_speed * sign_x,0f,0f);
        Vector3 new_pos_y = goal.transform.localPosition  + new Vector3(0f,goal_speed*sign_y,0f);
        float distance_x = Vector3.Distance(new_pos_x,arm_base.transform.localPosition);
        float distance_y = Vector3.Distance(new_pos_y,arm_base.transform.localPosition);
;        if( distance_x > ARM_RANGE - 0.1f || distance_x < 1.5f ){
            sign_x = -sign_x;   
        }
        if( distance_y > ARM_RANGE - 0.1f || arm_base.transform.localPosition.y -0.25f > new_pos_y.y || distance_y < 1.5f  ){
            sign_y = -sign_y;   
        }
        Vector3 increment = new Vector3(goal_speed * sign_x,goal_speed*sign_y,0f); 
        goal.transform.localPosition = increment + goal.transform.localPosition;

    }

    public void SpawnGoal(){
        m_Goal_phi = Random.Range(0, Mathf.PI);
        m_Goal_rho = Random.Range(2,ARM_RANGE);


        // 2D
        var goalX = m_Goal_rho * Mathf.Cos(m_Goal_phi);
        var goalY = m_Goal_rho * Mathf.Sin(m_Goal_phi);


        goal.transform.localPosition = new Vector3(goalX, goalY, 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
       
   
        sensor.AddObservation(segment1.transform.rotation);             //4    
        sensor.AddObservation(segment2.transform.rotation);             //4


        Vector3 goal_pos = goal.transform.position - ORIGIN;    // Ramène les coordonnées au centre de la scene
        Vector3 hand_pos = hand.transform.position - ORIGIN;    // Ramène les coordonnées au centre de la scene
        sensor.AddObservation(goal_pos);                    //3
        sensor.AddObservation(hand_pos);                    //3

    }

    // The agent's four actions correspond to torques on each of the two joints.
    public override void OnActionReceived(float[] vectorAction)
    {
        float reward_obtained;

        // Rotation du segment 1
        float consigne_1 = Mathf.Clamp(vectorAction[0], -1f, 1f) * 90;
        segment1.transform.rotation =  Quaternion.Euler(0f,0f,consigne_1);
        
        // Rotation du segment 2
        float consigne_2 = Mathf.Clamp(vectorAction[1], -1f, 1f) * 155 + consigne_1;
        segment2.transform.rotation =  Quaternion.Euler(0f,0f,consigne_2);
        consigne_1 = consigne_1 * Mathf.PI/180f;

        float pos_y = SEGMENT1_LENGTH * Mathf.Cos(consigne_1);
        float pos_x = SEGMENT1_LENGTH * Mathf.Sin(consigne_1);

        segment2.transform.localPosition = new Vector3(-pos_x,pos_y + 0.5f,0f);
        UpdateGoalPosition();

        float distance_to_target = Vector3.Distance(hand.transform.position,goal.transform.position);
        
        if(distance_to_target < positive_range){
            reward_obtained = 1/(distance_to_target * alpha);
        }
        else {
            reward_obtained = -1.0f  * distance_to_target;

        }
        SetReward(reward_obtained);
        episode_reward += reward_obtained;
        
    }



    public override float[] Heuristic()
    {
        var action = new float[2];
        action[0] = Input.GetAxis("Horizontal");
        action[1] = Input.GetAxis("Vertical");

        return action;
    }

    

    
}
