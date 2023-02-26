class HeadingTune
{
  constructor()
  {
    this.headingRequest = new ROSLIB.Topic({
      ros: ros,
      name : '/heading_request',
      messageType: 'std_msgs/msg/UInt32'
    });
  }

  request(mem)
  {
    var msg = new ROSLIB.Message({
      data: parseInt(mem)
    });
    this.headingRequest.publish(msg);
  }
}

var headingTuner = new HeadingTune();
