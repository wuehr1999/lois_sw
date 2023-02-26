class Visualizer
{
    constructor(divId, canvasId)
    {
      	this.divId = divId;
        this.canvasId = canvasId;
        this.canvas = null;
        this.context = null;
        this.width = 0;
        this.height = 0;

        this.plotting = false;
        this.odometry = false;
        this.map = false;
        this.osm = false;
        this.originX = 0;
        this.originY = 0;
        this.resolution = 1.0;
        this.lastGrid = null;

        this.track = null;
        this.posGPS = null;

        this.downloadData = null;
      	this.downloadMIME = null;
       	this.downloadName = null;
    
    }

    setCanvas()
    {
        this.canvas = document.getElementById(this.canvasId);
        this.context = this.canvas.getContext("2d");
        this.width = this.canvas.width;
        this.height = this.canvas.height;
    }


    showCanvas(show)
    {
	var mode = "block";
	if(!show)
        {
		mode = "none";
	}
	
	document.getElementById(this.canvasId).style.display = mode;
    }

    enableOdometry(show)
    {
      this.odometry = show;
    }

    enableMap(show)
    {
      this.map = show;
    }

    enableTrack(show)
    {
      this.osm = show;
    }

    show(name, type, message)
    {
        this.setCanvas();
        console.log(type);
        console.log(message);
        switch(type)
        {
            case "sensor_msgs/msg/LaserScan": this.showLaserScan(name, type, message); break;
            case "diagnostic_msgs/msg/DiagnosticStatus": this.showDiagnosticStatus(name, type, message); break;
            case "msgs/filteroutput": this.showFilter(name, type, message); break;
            //case "sensor_msgs/Image": this.showImage(name, type, message); break;
            case "sensor_msgs/msg/CompressedImage": this.showCompressedImage(name, type, message); break;
      	    case "std_msgs/msg/UInt16MultiArray": 
        			if(name.includes("/rpm_record"))
        			{
        				this.showRPMRecord(name, type, message);
        			}
        			else if(name.includes("/torque_record"))
        			{
        				this.showTorqueRecord(name, type, message);
        			}
        			else
        			{
        				this.showNotImplemented(name, type, message);
        			}
		          break;
            case "std_msgs/msg/Int16MultiArray":
              if(name.includes("/heading_record"))
              {
                this.showHeadingRecord(name, type, message);
              }
              else
              {
                this.showNotImplemented(name, type, message);
              }
              break;
            case "std_msgs/msg/Float64MultiArray":
              if(this.osm)
              {
                if("/route" == name)
                {
                  this.track = message;
                  this.visualizeTrack();
                }
              }
              break;
            case "std_msgs/msg/Int8": this.showInt(name, type, message); break;
            case "std_msgs/msg/Int16": this.showInt(name, type, message); break;
            case "std_msgs/msg/Int32": this.showInt(name, type, message); break;
            case "std_msgs/msg/Int64": this.showInt(name, type, message); break;
            case "std_msgs/msg/UInt8": this.showInt(name, type, message); break;
            case "std_msgs/msg/UInt16": this.showInt(name, type, message); break;
            case "std_msgs/msg/UInt32": this.showInt(name, type, message); break;
            case "std_msgs/msg/UInt64": this.showInt(name, type, message); break;
            case "std_msgs/msg/Float32": this.showInt(name, type, message); break;
            case "std_msgs/msg/Float64": this.showInt(name, type, message); break;
            case "sensor_msgs/msg/NavSatFix": 
              if(this.osm)
              {
                if(/*"/gps_data"*/"/ublox_gps/fix" == name)
                {
                  this.posGPS = message;
                  this.visualizeTrack();
                }
              }
              else
              {
                this.showNavSatFix(name, type, message); 
              }
              break;
            
            case "geometry_msgs/msg/Twist": this.showTwist(name, type, message); break;
  
            case "nav_msgs/msg/Odometry":
              if(!this.odometry && !this.map)
              {
                  this.showOdometry(name, type, message); 
              }
              else if(this.odometry)
              {
                var index = 0;
                if("/odom_wheel" == name)
                {
                  index = 0;
                }
                else if("/rtabmap/odom" == name)
                {
                  index = 1;
                }
                else if("/odometry/filtered" == name)
                {
                  index = 2;
                }
                this.showOdometryPath(name, type, message, index);
              }
              else if(this.map)
              {
                this.showOdomInGrid(name, type, message); 
              }
              break;
            case "nav_msgs/msg/OccupancyGrid": 
              this.showGrid(name, type, message); break;
            default: this.showNotImplemented(name, type, message); break;
        }
    }

    clearCanvas()
    {
      this.showCanvas(true);
      this.context.fillStyle = "#000000";
      this.canvas.height = this.height;
      this.canvas.width = this.width;
      this.context.scale(1, 1);
      this.context.fillRect(0, 0, this.width, this.height);
      this.context.setTransform(1, 0, 0, 1, 0, 0);
    }

    showNotImplemented(name, type, message)
    {
    	  this.showCanvas(true);
    	  console.log(name);
      	console.log(type);
    	  console.log(message);
        this.context.fillStyle = "#000000";
        this.context.fillRect(0, 0, this.width, this.height);
        this.context.fillStyle = "#ffffff";
        this.context.font = "30px Arial";
        this.context.fillText("No support for message type", 50, 50);
        this.context.fillText(type, 50, 100);
//        this.context.drawImage(document.getElementById("trollface"), 200, 225);
      this.defaultDownload(name);
    }

    showLaserScan(name, type, message)
    {
      	this.showCanvas(true);
        var centerX = this.width / 2;
        var centerY = this.height;
        var angle = 0;
        var max = message.ranges.reduce(function(a, b) {
            return Math.max(a, b);
        });
        var distToPixel = centerX / max;

        this.context.fillStyle = "#000000";
        this.context.fillRect(0, 0, this.width, this.height);
    
        this.context.strokeStyle = "#ff0000";
        this.context.beginPath();

        for(i in message.ranges)
        {
            var pointX = centerX + Math.cos(angle) * message.ranges[i] * distToPixel;
            var pointY = centerY - Math.sin(angle) * message.ranges[i] * distToPixel;
            this.context.moveTo(centerX, centerY);
            this.context.lineTo(pointX, pointY);
            angle += message.angle_increment;
        }
        this.context.stroke();
      this.defaultDownload(name);
    }

    showPointCloud(name, type, message)
    {
      this.showCanvas(true);
      if(1 == message.height)
      {
        var point = new Array(message.fields.length);
        var offset = 0;
        for(i in message.fields)
        {
          offset += message.fields[i].offset;
        }
        for(i = 0; i < message.data.length; i = i + offset)
        {
          for(f in message.fields)
          {

          }
        }
      }
      else
      {
        this.context.fillStyle = "#000000";
        this.context.fillRect(0, 0, this.width, this.height);
        this.context.fillStyle = "#ffffff";
        this.context.font = "30px Arial";
        this.context.fillText(name, 50, 50);
        this.context.fillText("No support for this pointclout type", 50, 100);
      }
    }

    showDiagnosticStatus(name, type, message)
    {
      	this.showCanvas(true);
        this.context.fillStyle = "#000000";
        this.context.fillRect(0, 0, this.width, this.height);
        this.context.fillStyle = "#ffffff";
        this.context.font = "30px Arial";
        this.context.fillText(name, 50, 50);
        var text;
        switch(message.level)
        {
            case 0: text = "OK"; break;
            case 1: text = "WARN"; break;
            case 2: text = "ERROR"; break;
            case 3: text = "STALE"; break;
            default: text = ""; break;
        }  
        this.context.fillText(text, 50, 100);  
      this.defaultDownload(name);
    }
    
    showFilter(name, type, message)
    {
      	this.showCanvas(true);
        this.context.fillStyle = "#000000";
        this.context.fillRect(0, 0, this.width, this.height);
        this.context.fillStyle = "#ffffff";
        this.context.font = "30px Arial";
        this.context.fillText(name, 50, 50);
        this.context.fillText("Offset: " + message.offset.data, 50, 100);  
        this.context.fillText("Blocked: " + message.noway.data, 50, 150);  
      this.defaultDownload(name);
    }

    showInt(name, type, message)
    {
      this.showCanvas(true);
      this.context.fillStyle = "#000000";
      this.context.fillRect(0, 0, this.width, this.height);
      this.context.fillStyle = "#ffffff";
      this.context.font = "30px Arial";
      this.context.fillText(name, 50, 50);
      this.context.fillText(message.data, 50, 100);
      this.defaultDownload(name);
    }
    
    showNavSatFix(name, type, message)
    {
      this.showCanvas(true);
      this.context.fillStyle = "#000000";
      this.context.fillRect(0, 0, this.width, this.height);
      this.context.fillStyle = "#ffffff";
      this.context.font = "30px Arial";
      this.context.fillText(name, 50, 50);
      this.context.fillText(message.latitude, 50, 100);
      this.context.fillText(message.longitude, 50, 150);
      this.defaultDownload(name);
    }

    showTwist(name, type, message)
    {
      this.showCanvas(true);
      this.context.fillStyle = "#000000";
      this.context.fillRect(0, 0, this.width, this.height);
      this.context.fillStyle = "#ff0000";
      this.context.font = "20px Arial";
      this.context.fillText(String(name), 50, 50);

      this.context.fillText("linear:", 50, 100);
      this.context.fillStyle = "#ffffff";
      this.context.fillText(String("x: " + message.linear.x.toFixed(3)), 50, 125);
      this.context.fillText(String("y: " + message.linear.y.toFixed(3)), 50, 150);
      this.context.fillText(String("z: " + message.linear.z.toFixed(3)), 50, 175);
      
      this.context.fillStyle = "#ff0000";
      this.context.fillText("angular:", 350, 100);
      this.context.fillStyle = "#ffffff";
      this.context.fillText(String("x: " + message.angular.x.toFixed(3)), 350, 125);
      this.context.fillText(String("y: " + message.angular.y.toFixed(3)), 350, 150);
      this.context.fillText(String("z: " + message.angular.z.toFixed(3)), 350, 175);
    }

    showOdometry(name, type, message)
    {
      this.showCanvas(true);
      this.context.fillStyle = "#000000";
      this.context.fillRect(0, 0, this.width, this.height);
      this.context.fillStyle = "#ff0000";
      this.context.font = "20px Arial";
      this.context.fillText(String(name + " (" + message.child_frame_id + ")"), 50, 50);
      
      this.context.fillText("pose:" , 50, 100);
      this.context.fillStyle = "#ffffff";
      this.context.fillText("orientation:", 50, 125);
      this.context.fillText(String("x: " + message.pose.pose.orientation.x.toFixed(3)), 50, 150);
      this.context.fillText(String("y: " + message.pose.pose.orientation.y.toFixed(3)), 50, 175);
      this.context.fillText(String("z: " + message.pose.pose.orientation.z.toFixed(3)), 50, 200);
      this.context.fillText(String("w: " + message.pose.pose.orientation.w.toFixed(3)), 50, 225);
      var q = message.pose.pose.orientation;
      var yaw = -Math.atan2(2.0*(q.w*q.z + q.w*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) * 180 / Math.PI;
      this.context.fillText(String("yaw: " + yaw.toFixed(3)), 50, 250);
      this.context.fillText("position:", 50, 275);
      this.context.fillStyle = "#ffffff";
      this.context.fillText(String("x: " + message.pose.pose.position.x.toFixed(3)), 50, 300);
      this.context.fillText(String("y: " + message.pose.pose.position.y.toFixed(3)), 50, 325);
      this.context.fillText(String("z: " + message.pose.pose.position.z.toFixed(3)), 50, 350);

      this.context.fillStyle = "#ff0000";
      this.context.fillText("twist:", 350, 100);
      this.context.fillStyle = "#ffffff";
      this.context.fillText("angular:", 350, 125);
      this.context.fillText(String("x: " + message.twist.twist.angular.x.toFixed(3)), 350, 150);
      this.context.fillText(String("y: " + message.twist.twist.angular.y.toFixed(3)), 350, 175);
      this.context.fillText(String("z: " + message.twist.twist.angular.z.toFixed(3)), 350, 200);
      this.context.fillText("linear:", 350, 275);
      this.context.fillText(String("x: " + message.twist.twist.linear.x.toFixed(3)), 350, 300);
      this.context.fillText(String("y: " + message.twist.twist.linear.y.toFixed(3)), 350, 325);
      this.context.fillText(String("z: " + message.twist.twist.linear.z.toFixed(3)), 350, 350);
      
      this.defaultDownload(name); 
    }

    showOdometryPath(name, type, message, index)
    {
      this.context.font = "20px Arial";
      var labelX = 10;
      var labelY = 15;
      var cols = [];
      cols[0] = "#ff0000";
      cols[1] = "#00ff00";
      cols[2] = "#0000ff";
      var col = cols[0];

      for(var i = 0; i <= index; i++)
      {
        col = cols[i];
        labelY += 25;
      }
      
      var pixelScale = 10;
      this.context.fillStyle = "#ffffff";
      this.context.fillText("1m", labelX, this.height - 25);
      this.context.fillRect(labelX, this.height - 22, pixelScale, 3);
      this.context.fillStyle = col;
      this.context.fillText(String(name), labelX, labelY);

      var x = message.pose.pose.position.x;
      var y = message.pose.pose.position.y;
      var q = message.pose.pose.orientation;
      var yaw = Math.atan2(2.0*(q.w*q.z + q.w*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) - 0.5 * Math.PI;
      
      var len = 20;
      
      x = -x * pixelScale;
      y = y * pixelScale;
      var dx = len * Math.sin(yaw);
      var dy = len * Math.cos(yaw);
      var offsetY = this.width / 2;
      var offsetX = this.height / 2;
      this.drawArrow(offsetY - y, x + offsetX, offsetY - y - dy, x + offsetX + dx, 1, 1, col);
      this.defaultDownload("odometryCompare");
    }

    showRPMRecord(name, type, message)
    {
	this.showCanvas(false);
	var half = Math.ceil(message.data.length / 2);
	var left = message.data.slice(0, half);
	var right = message.data.slice(-half);
	left[0] = 0;
	right[0] = 0;
	var xLeft = [];
	var yLeft = [];
	var xRight = [];
	var yRight = [];

	this.downloadData = "";
	this.downloadMIME = "text/xml";
	this.downloadName = "rpmlog.csv";

	var tMax = 0;

	for(i in left)
	{
    xLeft[i] = (left[i] * 100) / 1000 / 1000;
		xRight[i] = (right[i] * 100) / 1000 / 1000;
	
		this.downloadData += String(xLeft[i]) + ", " + String(xRight[i]) + "\n";
		yLeft[i] = 0;
		yRight[i] = 0;
		
	  yLeft[i] = 60 / (xLeft[i] * 60);
		yRight[i] = 60 / (xRight[i] * 60);
    if(i > 0)
    {
		  xLeft[i] += xLeft[i - 1];
		  xRight[i] += xRight[i - 1];
    }
	}

	if(xLeft[xLeft.length - 1] < xRight[xRight.length - 1])
	{
		tMax = xRight[xRight.length - 1];
	}
	else
	{
		tMax = xLeft[xLeft.length-  1];
	}

	var xDest = [];
	var yDest = [];
	var cnt = 0;

	for(var t = 0; t < tMax; t += 0.1)
	{
		xDest[cnt] = t;
		yDest[cnt] = tuner.getDest(t);
		cnt++;
	}

	var layout = {
		title: "RPM measurement",
		showlegend: true,
		xaxis: {
			title: "t/sec"
		},
		yaxis: {
			title: "RPM"
		}
	};

  console.log(xLeft, xRight, yLeft, yRight);
	var data = [{
		name: "left",
		x: xLeft,
		y: yLeft,
		mode: 'lines',
		connectgaps: true,
		line: {
			color: 'rgb(255, 0, 0)'
		}
		},{
		name: "right",
		x: xRight,
		y: yRight,
		mode: 'lines',
		connectgaps: true,
		line: {
			color: 'rgb(0, 0, 255)'
		
		}},{
		name: "dest",
		x: xDest,
		y: yDest,
		mode: 'lines',
		connectgaps: true,
		line: {
			color: 'rgb(0, 255, 0)'
		}
	}];
	Plotly.newPlot(this.divId, data, layout);
	}
  
  showTorqueRecord(name, type, message)
  {
  	this.showCanvas(false);
	  var half = Math.ceil(message.data.length / 2);
  	var left = message.data.slice(0, half);
	  var right = message.data.slice(-half);
  	left[0] = 0;
	  right[0] = 0;
	var xLeft = [];
	var yLeft = [];
	var xRight = [];
	var yRight = [];

	this.downloadData = "";
	this.downloadMIME = "text/xml";
	this.downloadName = "currentlog.csv";

	var tMax = 0;

	for(i in left)
	{
    xLeft[i] = i * bot.INTERVAL_RPMCTRL_MS * 0.001 ;
    xRight[i] = i * bot.INTERVAL_RPMCTRL_MS * 0.001 ;
    yLeft[i] = left[i];
    yRight[i] = right[i];
	}

	if(xLeft[xLeft.length - 1] < xRight[xRight.length - 1])
	{
		tMax = xRight[xRight.length - 1];
	}
	else
	{
		tMax = xLeft[xLeft.length-  1];
	}

	var xDest = [];
	var yDest = [];
	var cnt = 0;

	for(var t = 0; t < tMax; t += 0.1)
	{
		xDest[cnt] = t;
		yDest[cnt] = tuner.getDest(t);
		cnt++;
	}

	var layout = {
		title: "Torque measurement",
		showlegend: true,
		xaxis: {
			title: "t/sec"
		},
		yaxis: {
			title: "Current/12 bit"
		}
	};

  console.log(xLeft, xRight, yLeft, yRight);
	var data = [{
		name: "left",
		x: xLeft,
		y: yLeft,
		mode: 'lines',
		connectgaps: true,
		line: {
			color: 'rgb(255, 0, 0)'
		}
		},{
		name: "right",
		x: xRight,
		y: yRight,
		mode: 'lines',
		connectgaps: true,
		line: {
			color: 'rgb(0, 0, 255)'
		
		}
  	}];
	Plotly.newPlot(this.divId, data, layout);
	}

  showHeadingRecord(name, type, message)
  {
    this.showCanvas(false);
    var x = [];
    var y = message.data;
  
	  this.downloadData = "";
  	this.downloadMIME = "text/xml";
  	this.downloadName = "headinglog.csv";

    for(var i = 0; i < y.length; i++)
    {
      x[i] = i;
      if(y[i] < 0)
      {
        y[i] = y[i] + 360;
      }
		  this.downloadData += String(x[i]) + ", " + String(y[i]) + "\n";
    }

    var layout = {
      title: "Heading Record",
      showlegend: true,
      xaxis: {
        title: "samples"
      },
      yaxis: {
        title: "heading/ deg"
      }
    };
          
    var data = [{
      name: "KVH-C100",
      x: x,
      y: y,
      mode: 'lines',
      connectgaps: true,
      line: {
        color: 'rgb(255, 0, 0)'
      }
    }];
    Plotly.newPlot(this.divId, data, layout);
  }
          
    showImage(name, type, message)
    {
          this.showCanvas(true);
          this.context.fillStyle = "#000000";
          this.context.fillRect(0, 0, this.width, this.height);
          console.log(message);
          var width = message.width;
          var height = message.height;
          var bytes = message.step / width

          var stepsX = 1;
          var stepsY = 1;

          console.log("width: " + width + "height: " + height + "bytes: " + bytes);

          var img = this.context.getImageData(0, 0, this.width, this. height);
          var pixels = img.data;
  
          var enc = new TextEncoder();
          var arr = enc.encode(message.data);
          console.log(arr.length); 
          for(var y = 0; y < height; y ++)
          {
            for(var x = 0; x < width; x ++)
            {
                if(x < this.width && y < this.height)
                {
                var idxOut = 4 * (x + y * this.width);
                var idxIn = bytes * (x + y * width);
                var inVal = 0;
                var val = 0;
                for(var b = 0; b < 3; b++)
                {
                  if(b < bytes)
                  {
                    val += arr[idxIn + b];
                  }
                  else
                  {
                    pixels[idxOut + b] = 0;
                  }

                }
                pixels[idxOut] = val;
                pixels[idxOut + 1] = 0;
                pixels[idxOut + 2] = 0;
                pixels[idxOut + 3] = 255;
                }
            }
          }
//       console.log(pixels); 
      this.context.putImageData(img, 0, 0);
      this.defaultDownload(name);
    }

    showGrid(name, type, message)
    {
      console.log(message);
      this.lastGrid = message;
      this.showCanvas(true);
      this.clearCanvas(true);
      var width = message.info.width;
      var height = message.info.height;
      var arr = message.data;

    var facX = 1;
    var facY = 1;
    if(height > this.height)
    {
      this.canvas.height = height;
      var facY = this.height / height;
    }
    if(width > this.width)
    {
      this.canvas.width = width;
      var facX = this.width / width;
    }

    var img = this.context.getImageData(0, 0, width, height);
    var pixels = img.data;

    for(var y = 0; y < height; y++)
    {
      for(var x = 0; x < width; x++)
      {
        var idxOut = 4 * (x + y * width);
        var idxIn = (x + (height - y) * width);
        if(-1 == arr[idxIn])
        {
          pixels[idxOut] = 0;
          pixels[idxOut + 1] = 0;
          pixels[idxOut + 2] = 50;
        }
        else
        {
          pixels[idxOut] = 0;
          pixels[idxOut + 2] = 0;
          pixels[idxOut + 1] = parseInt(255 * arr[idxIn] * 0.01);
        }
        pixels[idxOut + 3] = 255;
      }
    }

    var pose = message.info.origin;
    
    this.context.putImageData(img, 0, 0);
    
    var scale = Math.min(facX, facY);
//    this.context.scale(scale, scale);

    this.originX = parseFloat(pose.position.x).toFixed(2);
    this.originY = parseFloat(pose.position.y).toFixed(2);
    this.resolution = parseFloat(message.info.resolution).toFixed(2);
    this.context.font = "15px Arial";
    this.context.fillStyle = "#ffffff";
    this.context.fillText("m/cell: " + String(parseFloat(message.info.resolution).toFixed(2)) +
    ", P0(" + String(parseFloat(pose.position.x).toFixed(2)) + "|" + String(parseFloat(pose.position.y).toFixed(2)) + "), Size: " + String(width) + "x" + String(height), 100, 25);

    this.defaultDownload(name);
  }

  showOdomInGrid(name, type, message)
  {
    this.showGrid("/grid", "nav_msgs/OccupancyGrid", this.lastGrid);
    let x = message.pose.pose.position.x / this.resolution;
    let y = message.pose.pose.position.y / this.resolution;
    let q = message.pose.pose.orientation;
    let yaw = Math.atan2(2.0*(q.w*q.z + q.w*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) + 0.5 * Math.PI;
    console.log(message.pose.pose.position.x, message.pose.pose.position.y, this.originX, this.originY);
    x += -this.originX / this.resolution;
    y += -this.originY / this.resolution;
    y = this.lastGrid.info.height - y;
    var dx = 30 * Math.sin(yaw);
    var dy = 30 * Math.cos(yaw);
    console.log(x, y);
    this.drawArrow(x, y, x + dx, y + dy, 1, 3, "#ff0000");
    this.defaultDownload("map");
  }
  
  showCompressedImage(name, type, message)
    {
      this.showCanvas(true);
      this.context.fillStyle = "#000000";
      this.context.fillRect(0, 0, this.width, this.height);
      var image = "data:image/jpg;base64," + message.data;
      var img = new Image();
      img.src=image;
      img.onload = () =>{
      var w = img.width;
      var h = img.height;
      var sizer = Math.min((this.width/w), (this.height/h));
      this.context.drawImage(img, 0, 0, w, h, 0, 0, w * sizer, h * sizer);
      this.defaultDownload(name);}
    }

    visualizeTrack()
    {
      this.clearCanvas();
      if(this.track)
      {
        var t = new Track();
        for(var i = 0; i < this.track.data.length - 2; i++)
        {
          t.add(new Trackpoint(this.track.data[2 * i], this.track.data[2 * i + 1], 0, 0, 0));
        }
       this.context.strokeStyle = "#ff0000";
       this.context.beginPath();

       var upperLeftPoint = t.getUpperLeft();
       var lowerRightPoint = t.getLowerRight();
       if(this.posGPS)
       {
        if(this.posGPS.latitude < upperLeftPoint.latitude)
        {
          upperLeftPoint.latitude = this.posGPS.latitude;
        }
        if(this.posGPS.latitude > lowerRightPoint.latitude)
        {
          lowerRightPoint.latitude = this.posGPS.latitude;
        }
        if(this.posGPS.longitude > lowerRightPoint.longitude)
        {
          lowerRightPoint.longitude = this.posGPS.longitude;
        }
        if(this.posGPS.longitude < upperLeftPoint.longitude)
        {
          upperLeftPoint.longitude = this.posGPS.longitude;
        }
      }
       var dx = upperLeftPoint.calculateDistance(new Trackpoint(upperLeftPoint.lat, lowerRightPoint.lon, 0, 0, 0));
       var dy = upperLeftPoint.calculateDistance(new Trackpoint(lowerRightPoint.lat, upperLeftPoint.lon, 0, 0, 0));

       var d = dx;
       if(d < dy)
       {
         d = dy;
       }

       var offset = 30;
       var metersPerPixel = d / (this.height - 2 * offset);

       for(var i = 0; i < t.getNumberOfPoints() - 1; i++)
       {
         var p1 = t.points[i];
         var p2 = t.points[i + 1];
           
         var c1 = p1.xyPixelsFromStartPoint(upperLeftPoint, metersPerPixel);
         var c2 = p2.xyPixelsFromStartPoint(upperLeftPoint, metersPerPixel);
         this.context.moveTo(c1[0] + offset, c1[1] + offset);
         this.context.lineTo(c2[0] + offset, c2[1] + offset);
       }

       this.context.stroke();
       
       if(this.posGPS)
       {
//         if(-1000 != this.posGPS.latitude && -1000 != this.posGPS.longitude)
//         {
           var cur = new Trackpoint(this.posGPS.latitude, this.posGPS.longitude, 0, 0, 0);
//           var cur = new Trackpoint(49.0012894, 12.8279123, 0, 0, 0);
           this.context.fillStyle = "#00ff00";
           var coords = cur.xyPixelsFromStartPoint(upperLeftPoint, metersPerPixel);
           this.context.beginPath();
           this.context.arc(coords[0] + offset, coords[1] + offset, 5, 0, 2 * Math.PI);
           this.context.fill();
//         }
       }
      }
      this.defaultDownload("map");
    }
    drawArrow(fromx, fromy, tox, toy, lineWidth, headLength, style)
    {
//      console.log("Arrow, " + String(fromx) + ", " + String(fromy) + ", " + String(tox) + ", " + String(toy));
      var ctx = this.context;
      const width = lineWidth;
      var headlen = headLength;

      var angle = Math.atan2(toy-fromy,tox-fromx);      
      tox -= Math.cos(angle) * ((width*1.15));
      toy -= Math.sin(angle) * ((width*1.15));

      this.context.beginPath();
      this.context.moveTo(fromx, fromy);
      this.context.lineTo(tox, toy);
      this.context.strokeStyle = style;
      this.context.lineWidth = width;
      this.context.stroke();

      ctx.beginPath();
      ctx.moveTo(tox, toy);
      ctx.lineTo(tox-headlen*Math.cos(angle-Math.PI/7),toy-headlen*Math.sin(angle-Math.PI/7));

      ctx.lineTo(tox-headlen*Math.cos(angle+Math.PI/7),toy-headlen*Math.sin(angle+Math.PI/7));
      ctx.lineTo(tox, toy);
      ctx.lineTo(tox-headlen*Math.cos(angle-Math.PI/7),toy-headlen*Math.sin(angle-Math.PI/7));

      ctx.strokeStyle = style;
      ctx.lineWidth = width;
      ctx.stroke();
      ctx.fillStyle = style;
      ctx.fill();
    }

    showIdle()
    {
      	this.showCanvas(true);
        if(this.context != null)
        {
            this.context.fillStyle = "#000000";
            this.context.fillRect(0, 0, this.width, this.height);
            this.context.fillStyle = "#ffffff";
            this.context.font = "30px Arial";
            this.context.fillText("Waiting for next callback", 50, 50);       
        }
    }

    defaultDownload(name)
    {
      this.downloadName = name;
      this.downloadMIME = "image/jpeg";
      this.downloadData = this.canvas.toDataURL("image/jpeg");
    }
    
    download()
    {
      	if(null != this.downloadData)
        {
	        console.log(this.downloadData);
    	    download(this.downloadData, this.downloadName, this.downloadMIME);
        }
    }
}

//var visualizer = new Visualizer("div_screen", "canvas_screen");

