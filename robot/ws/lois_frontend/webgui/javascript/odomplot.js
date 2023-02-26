var ros;

var data;

var names = [ "/odom_wheel" , "/rtabmap/odom"];

function init()
{
    document.addEventListener('DOMContentLoaded', init, false);	
    var wsUri = "ws://"+location.hostname+":9090";

    ros = new ROSLIB.Ros({
      url : wsUri
    });
}

function subscribe(name)
{
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : name, 
      messageType : 'nav_msgs/Odometry'
    });
    
    listener.subscribe(function(message) {

      console.log(message);
      var x = message.pose.pose.position.x;
      var y = message.pose.pose.position.y;
      var q = message.pose.pose.orientation;
      var yaw = Math.atan2(2.0*(q.w*q.z + q.w*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
      yaw = yaw / Math.PI * 180;
      
      if(data[name]["t"].length > 0)
      {
        data[name]["t"].push(message.header.stamp.secs - data[name]["t"][0]);
      }
      else
      {
        data[name]["t"].push(message.header.stamp.secs);
      }
      data[name]["x"].push(x);
      data[name]["y"].push(y);
      data[name]["yaw"].push(yaw);
      console.log(data[name]["y"].length);
    })
}

function button_start_onclick()
{
    data = null;
    data = [];
    data = new Array(names.length);
    console.log(names);

    for(n in names)
    {
      console.log(n);
      data[names[n]] = new Array(4);  
      data[names[n]]["t"] = new Array();
      data[names[n]]["x"] = new Array();  
      data[names[n]]["y"] = new Array();  
      data[names[n]]["yaw"] = new Array();  
      subscribe(names[n]);
    }
}

function button_stop_onclick()
{
	var coltable = ['rgb(255, 0, 0)', 'rgb(0, 255, 0)', 'rgb(0, 0, 255)']
  ;
  var layout = [
  {
		title: "X",
		showlegend: true,
		xaxis: {
			title: "t/sec"
		},
		yaxis: {
			title: "x/m"
		}
	},
  {
		title: "Y",
		showlegend: true,
		xaxis: {
			title: "t/sec"
		},
		yaxis: {
			title: "y/m"
		}
	},
  {
		title: "Yaw",
		showlegend: true,
		xaxis: {
			title: "t/sec"
		},
		yaxis: {
			title: "yaw/deg"
		}
	}];
	
	var plotData = new Array(3);
  plotData[0] = new Array();
  plotData[1] = new Array();
  plotData[2] = new Array();

  for(n in names)
  {
    data[names[n]]["t"][0] = 0.0;
    plotData[0].push({
      name: names[n],
      x: data[names[n]]["t"],
      y: data[names[n]]["x"],
      connectgaps: true,
      line: {
        color: coltable[n]
      }});
    plotData[1].push({
      name: names[n],
      x: data[names[n]]["t"],
      y: data[names[n]]["y"],
      connectgaps: true,
      line: {
        color: coltable[n]
      }});
    plotData[2].push({
      name: names[n],
      x: data[names[n]]["t"],
      y: data[names[n]]["yaw"],
      connectgaps: true,
      line: {
        color: coltable[n]
      }});
  }
  console.log(plotData);	
  Plotly.newPlot("xplot", plotData[0], layout[0]);
  Plotly.newPlot("yplot", plotData[1], layout[1]);
  Plotly.newPlot("yawplot", plotData[2], layout[2]);
}
