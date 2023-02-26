class EmergencyStop
{
    constructor()
    {
        this.pressed = false;

        this.emListener = new ROSLIB.Topic({
            ros : ros,
            name : '/emergency_stop',
            messageType : 'diagnostic_msgs/msg/DiagnosticStatus'
        });
        this.subscribe();
        this.emPublisher = new ROSLIB.Topic({
            ros : ros,
            name : '/emergency_stop',
            messageType : 'diagnostic_msgs/msg/DiagnosticStatus'
        });
    }
    
    press()
    {
        this.pressed = !this.pressed;
        this.publish();
        button_emstop_change(this.pressed)
    }

    isPressed()
    {
        return this.pressed;
    }

    isFromOther()
    {
        return this.fromOther;
    }

    subscribe()
    {

        function handle(message)
        {
            if(message.level == 2)
            {
                emergencyStop.pressed = true;
            }
            else
            {
                emergencyStop.pressed = false;
            }
            button_emstop_change(emergencyStop.pressed);
        }

        this.emListener.subscribe(function(message) {
            handle(message);
        });

    }

    publish()
    {
        var state;
        if(this.pressed)
        {
            state = 2;
        }
        else
        {
            state = 0;
        }

        var em = new ROSLIB.Message({
            level : state
        });
        this.emPublisher.publish(em);
    }
}

var emergencyStop = new EmergencyStop();
console.log("created");
