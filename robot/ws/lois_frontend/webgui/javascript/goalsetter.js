var xOdom, yOdom, headingOdom;
var headingKVH, headingGoal;

class GoalSetter
{
  constructor()
  {
    this.actionClient = new ROSLIB.ActionClient({
      ros : ros,
      serverName : '/move_base',
      actionName : 'move_base_msgs/msg/MoveBaseAction'
    });

    this.send = false;
    this.goal = null;
/*    this.xOdom = 0; 
    this.yOdom = 0;
    this.headingOdom = 0;
*/
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/odom_slam', 
      messageType : 'nav_msgs/msg/Odometry'
    });
    
    listener.subscribe(function(message) {

      xOdom = message.pose.pose.position.x;
      yOdom = message.pose.pose.position.y;
      console.log(xOdom, yOdom)
      var q = message.pose.pose.orientation;
      headingOdom = Math.atan2(2.0*(q.w*q.z + q.w*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    })

    var listenerKVH = new ROSLIB.Topic({
      ros : ros,
      name : '/kvh_heading',
      messageType : 'std_msgs/msg/Int32'
    });

    listenerKVH.subscribe(function(message) {
      headingKVH = message.data;
    });

    this.globalGoal = new ROSLIB.Topic({
      ros : ros,
      name : '/globalgoal',
      messageType : 'msgs/msg/loisgoal'
     });
  }

  request(distance, heading)
  {
    this.send = true;
    headingGoal = heading + headingKVH;
    this.request2(distance, headingGoal);
  }
  
  request2(distance, heading)
  {
    heading = heading - headingKVH; 
    var positionVec3 = new ROSLIB.Vector3(null);
    console.log(xOdom, yOdom);
    positionVec3.x = xOdom + Math.cos(headingOdom - heading / 180 * Math.PI) * distance;
    positionVec3.y = yOdom + Math.sin(headingOdom - heading / 180 * Math.PI) * distance;
    console.log("x " + String(positionVec3.x) + ", y" + String(positionVec3.y)); 
    var orientation = new ROSLIB.Quaternion({x:0, y:0, z:0, w:1.0});
    var pose = new ROSLIB.Pose({
      position : positionVec3,
      orientation : orientation
    });
    
    this.goal = new ROSLIB.Goal({
      actionClient : this.actionClient,
      goalMessage : {
        target_pose : {
          header : {
            frame_id : 'odom'
          },
          pose : pose
        }
      }
    });
    
/*    var payload = {} 
    payload["distance_m"] = {}
    payload["distance_m"]["data"] = parseFloat(distance)
    payload["heading_deg"] = {}
    payload["heading_deg"]["data"] = parseFloat(heading)
    var msg = new ROSLIB.Message({
      data: payload
    i});*/
    if(heading < -180.0)
    {
      heading += 360.0;
    }
    else if(heading > 180.0)
    {
      heading -= 360.0;
    }
    var heading = new ROSLIB.Message({
      data: parseFloat(heading)
    });
    var dist = new ROSLIB.Message({
      data: parseFloat(distance)
    });

    var msg = new ROSLIB.Message({
      heading_deg: heading,
      distance_m: dist
    });
    console.log(msg)
    this.globalGoal.publish(msg)
    console.log("published goal")
//    this.goal.send();
  }

  cancel()
  {
      this.send = false;
      this.goal.cancel();
  }
}

var goalSetter = new GoalSetter();
let goalTimer = setInterval(
  function() {
    if(goalSetter.send)
    {
      //goalSetter.request2(100.0, headingGoal);
    }
  },
  1000
);
