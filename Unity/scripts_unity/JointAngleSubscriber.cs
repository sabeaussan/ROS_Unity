

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class JointAngleSubscriber : UnitySubscriber<MessageTypes.Std.Float32MultiArray>
    {
        public Transform SubscribedTransform;
		public int id;
		public int axis_x; 
		public int axis_y;
		public int axis_z;
		private int robot_speed = 2;
		
        private float desired_angle;
        private float new_angle;
		private int sign;
		private bool stop;
		private float oldDesiredAngle = 0f;

        protected override void Start()
        {
			base.Start();
			stop = true;
			new_angle=0f;
		}
		
        private void Update()
        {
            if(!stop)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Std.Float32MultiArray message)
        {
            desired_angle = message.data[id];//.Ros2Unity();
			stop = oldDesiredAngle == desired_angle;
        }

        private void ProcessMessage()
        {	
			if(new_angle < desired_angle ){
				sign = 1;
			}
			else{
				sign = -1;
			}
			if(id == 4) {
				Debug.Log( "id : "+id+ " sign : " +sign);
			}
			/*SubscribedTransform.Rotate(
					robot_speed * axis_x * sign,
					robot_speed * axis_y * sign,
					robot_speed * axis_z * sign
			);*/
			new_angle = new_angle + robot_speed * sign;
			SubscribedTransform.localRotation = Quaternion.Euler(new_angle * axis_x ,new_angle * axis_y, axis_z * new_angle);
			if(id == 4) {
				Debug.Log( "id : "+id+ " new_angle : " +new_angle);
				Debug.Log( "id : "+id+ " sign : " +sign);
			}
			if(sign == 1){
				stop = new_angle >= desired_angle;
				oldDesiredAngle = desired_angle;
			}
			else{
				stop = new_angle <= desired_angle;
				oldDesiredAngle = desired_angle;
			}
			if(id == 4)
				Debug.Log( "id : "+id+ " stop : " +stop);
        }

    }
} 