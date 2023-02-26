var ros;

var parameterMsg;

function init()
{
    document.addEventListener('DOMContentLoaded', init, false);	
    var wsUri = "ws://"+location.hostname+":9090";

    ros = new ROSLIB.Ros({
      url : wsUri
    });

    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/ecu_params',
      messageType : 'lois_ecu/msg/RuntimeParameters'
    });
   
    listener.subscribe(function(message) {
      parameterMsg =  message;
      console.log(parameterMsg);
      console.log(`Received message on  ${listener.name}: ${JSON.stringify(message)}`);
      document.getElementById("terminal_mode").value = message["terminal_mode"]["data"];
      document.getElementById("rpmctrl_enable").value = message["rpmctrl_enable"]["data"];
      document.getElementById("rpm_lowpass").value = message["rpm_lowpass"]["data"];
      document.getElementById("ka_left").value = message["ka_left"]["data"];
      document.getElementById("kp_left").value = message["kp_left"]["data"];
      document.getElementById("tn_left").value = message["tn_left"]["data"];
      document.getElementById("td_left").value = message["td_left"]["data"];
      document.getElementById("ka_right").value = message["ka_right"]["data"];
      document.getElementById("kp_right").value = message["kp_right"]["data"];
      document.getElementById("tn_right").value = message["tn_right"]["data"];
      document.getElementById("td_right").value = message["td_right"]["data"];
      document.getElementById("corr_long_left").value = message["corr_long_left"]["data"];
      document.getElementById("corr_short_left").value = message["corr_short_left"]["data"];
      document.getElementById("corr_long_right").value = message["corr_long_right"]["data"];
      document.getElementById("corr_short_right").value = message["corr_short_right"]["data"];
      document.getElementById("period_latlon").value = message["period_latlon"]["data"];
      document.getElementById("period_date").value = message["period_date"]["data"];
      document.getElementById("period_time").value = message["period_time"]["data"];
      document.getElementById("period_heading").value = message["period_heading"]["data"];
      document.getElementById("period_encoders").value = message["period_encoders"]["data"];
      document.getElementById("period_odometry").value = message["period_odometry"]["data"];
   });

  button_load_onclick();
}

function publishParameters(save)
{
    var pub = new ROSLIB.Topic({
      ros : ros,
      name : '/ecu_params',
      messageType : 'lois_ecu/msg/RuntimeParameters'
    });

    
    parameterMsg["terminal_mode"]["data"] = parseInt(document.getElementById("terminal_mode").value);
    parameterMsg["rpmctrl_enable"]["data"] = parseInt(document.getElementById("rpmctrl_enable").value);
    parameterMsg["rpm_lowpass"]["data"] = parseFloat(document.getElementById("rpm_lowpass").value);
    parameterMsg["ka_left"]["data"] = parseFloat(document.getElementById("ka_left").value);
    parameterMsg["kp_left"]["data"] = parseFloat(document.getElementById("kp_left").value);
    parameterMsg["tn_left"]["data"] = parseFloat(document.getElementById("tn_left").value);
    parameterMsg["td_left"]["data"] = parseFloat(document.getElementById("td_left").value);
    parameterMsg["ka_right"]["data"] = parseFloat(document.getElementById("ka_right").value);
    parameterMsg["kp_right"]["data"] = parseFloat(document.getElementById("kp_right").value);
    parameterMsg["tn_right"]["data"] = parseFloat(document.getElementById("tn_right").value);
    parameterMsg["td_right"]["data"] = parseFloat(document.getElementById("td_right").value);
    parameterMsg["corr_long_left"]["data"] = parseFloat(document.getElementById("corr_long_left").value);
    parameterMsg["corr_short_left"]["data"] = parseFloat(document.getElementById("corr_short_left").value);
    parameterMsg["corr_long_right"]["data"] = parseFloat(document.getElementById("corr_long_right").value);
    parameterMsg["corr_short_right"]["data"] = parseFloat(document.getElementById("corr_short_right").value);
    parameterMsg["period_latlon"]["data"] = parseInt(document.getElementById("period_latlon").value);
    parameterMsg["period_date"]["data"] = parseInt(document.getElementById("period_date").value);
    parameterMsg["period_time"]["data"] = parseInt(document.getElementById("period_time").value);
    parameterMsg["period_heading"]["data"] = parseInt(document.getElementById("period_heading").value);
    parameterMsg["period_encoders"]["data"] = parseInt(document.getElementById("period_encoders").value);
    parameterMsg["period_odometry"]["data"] = parseInt(document.getElementById("period_odometry").value);
    parameterMsg["save"]["data"] = save;

    console.log(parameterMsg);
    pub.publish(parameterMsg);
}

function button_load_onclick()
{
  var request = new ROSLIB.Topic({
    ros: ros,
    name: 'ecu_params_request',
    messageType: 'std_msgs/msg/Bool'
  });

  var rq = new ROSLIB.Message({
    data : true
  });

  request.publish(rq);
}

function button_apply_onclick()
{
  publishParameters(false);
}

function button_save_onclick()
{
  publishParameters(true);
}
