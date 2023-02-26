class Joydrive
{
    constructor(maxX, maxR, xStep, rStep)
    {
        this.maxX = maxX;
        this.maxR = maxR;
        this.xStep = xStep;
        this.rStep = rStep;

        this.vX = 0.0;
        this.vR = 0.0;

        this.cmdVel = new ROSLIB.Topic({
            ros : ros,
            name : '/cmd_vel',
            messageType : 'geometry_msgs/msg/Twist'
        });

        this.shitter = new ROSLIB.Topic({
            ros : ros,
            name : '/shit_ball',
            messageType : 'std_msgs/msg/UInt32'
        });
    }

    publish()
    {
        if(emergencyStop.isPressed())
        {
            this.vX = 0.0;
            this.vR = 0.0;
        }
        var twist = new ROSLIB.Message({
            linear : {
                x : this.vX,
                y : 0.0,
                z : 0.0
            },
            angular : {
                x : 0.0,
                y : 0.0,
                z : this.vR
            }
        });
        this.cmdVel.publish(twist);
    }

    shit()
    {
      var msg = new ROSLIB.Message({
        data: parseInt(1)
      });
      this.shitter.publish(msg);
    }

    up()
    {
        this.vX += this.xStep;
        if(this.vX > this.maxX)
        {
            this.vX = this.maxX;
        }

        this.publish();
    }

    down()
    {
        this.vX -= this.xStep;
        if(this.vX < -this.maxX)
        {
            this.vX = -this.maxX;
        }

        this.publish();
    }

    right()
    {
        this.vR += this.rStep;
        if(this.vR > this.maxR)
        {
            this.vR = this.maxR;
        }

        this.publish();
    }

    left()
    {
        this.vR -= this.rStep;
        if(this.vR < -this.maxR)
        {
            this.vR = -this.maxR;
        }

        this.publish();
    }

    stop()
    {
        this.vR = 0.0;
        this.vX = 0.0;
        this.publish();
    }
}

var joydrive = new Joydrive(2.0, 2.0, 0.1, 0.1);
