class CtrlTune
{
    constructor(vMax, interval_ms)
    {
	this.vMax = vMax;
	this.step = 0;
	this.mode = "";
	this.interval_ms = interval_ms;
	this.timer = null;
	this.time = 0;

	this.functions = [];
	this.functions["dutycyclestep"] = function(sec, step) { return step; };
	this.functions["ctrlstep"] = function(sec, step) { return step; };
	this.functions["ctrlspeed"] = function(sec, step) { return sec * step; };
	this.functions["ctrlacc"] = function(sec, step) { return Math.pow(step, sec); };

        this.enableCtrl = new ROSLIB.Topic({
            ros : ros,
            name : '/enable_rpmctrl',
            messageType : 'std_msgs/msg/Bool'
        });
        
        this.rpmRequest = new ROSLIB.Topic({
            ros : ros,
            name : '/rpm_request',
            messageType : 'std_msgs/msg/UInt16MultiArray'
        });
        
        this.torqueRequest = new ROSLIB.Topic({
            ros : ros,
            name : '/torque_request',
            messageType : 'std_msgs/msg/UInt16MultiArray'
        });

        this.cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : '/cmd_vel',
            messageType : 'geometry_msgs/msg/Twist'
        });
    }

    getDest(sec)
    {
      var val = this.functions[this.mode](sec, this.step);
      if(val > bot.RPMMAX)
      {
        val = bot.RPMMAX;
      }
      return val;
    }

    requestRPM(memLeft, memRight)
    {
        var msg = new ROSLIB.Message({
            data: [parseInt(memLeft), parseInt(memRight)]
        });
        this.rpmRequest.publish(msg);
    }
    
    requestTorque(memLeft, memRight)
    {
        var msg = new ROSLIB.Message({
            data: [parseInt(memLeft), parseInt(memRight)]
        });
        this.torqueRequest.publish(msg);
    }

    enableControl(enable)
    {
        var msg = new ROSLIB.Message({
            data: enable
        });
        this.enableCtrl.publish(msg);
    }

    execute()
    {
	tuner.time += tuner.interval_ms;
	var val = tuner.getDest(tuner.time / 1000);
	if(!tuner.mode.includes("dutycycle"))
	{
		val = (val / bot.ENCODER_STEPS) * 2 * Math.PI * bot.WHEEL_RADIUS;
	}
	tuner.v0(val);
    }

    doSequence(mode, memDepth, step, rpm)
    {
	this.step = step;
	this.mode = mode;
      if(rpm)
      {
        this.requestRPM(memDepth, memDepth);
      }
      else
      {
        this.requestTorque(memDepth, memDepth);
      }

        if("dutycyclestep" == mode)
        {
            //this.stepDutycycle(step);
	    this.enableControl(false);
        }
        else if("ctrlstep" == mode)
        {
            //this.stepCtrl(step);
	    this.enableControl(true);
        }
	
	this.time = 0;
	this.timer = setInterval(this.execute, this.interval_ms);
    }

    stepDutycycle(step)
    {
        this.enableControl(false);
        this.v0(step);
    }

    stepCtrl(step)
    {
        this.enableControl(true);
        this.v0(step * 0.01 * this.vMax);
    }

    v0(speed)
    {
        var twist = new ROSLIB.Message({
            linear : {
                x : parseFloat(speed),
                y : 0.0,
                z : 0.0
            },
            angular : {
                x : 0.0,
                y : 0.0,
                z : 0.0
            }
        });
        this.cmdVel.publish(twist);
    }

    reset()
    {
	clearInterval(this.timer);
        this.v0(0);
        this.enableControl(true);
    }
}

var tuner = new CtrlTune(bot.VMAX, bot.INTERVAL_RPMCTRL_MS);
