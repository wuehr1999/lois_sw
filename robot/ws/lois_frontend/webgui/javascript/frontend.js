var opMode = "joydrive";
var odomShow = 0;
var mapShow = 0;
var trackShow = 0;

function button_up_onclick() {
    joydrive.up();
}

function button_down_onclick() {
    joydrive.down();
}

function button_left_onclick() {
    joydrive.left();
}

function button_right_onclick() {
    joydrive.right();
}

function button_shit_onclick() {
    joydrive.shit();
}

function button_emstop_onclick() {
    emergencyStop.press();
}

function button_emstop_change(pressed) {
    if(pressed)
    {
        joydrive.stop();
    }
    var btn = document.getElementById("button_emstop");
    if (btn == null) {
        return;
    }

    if (pressed) {
        btn.style.backgroundColor = 'yellow';
    }
    else {
        btn.style.backgroundColor = 'red';
    }
}

function prepareJoydrive(show) {
    var style = "none";
    if(show)
    {
        style = "block";
        prepareCtrlTune(false);
        prepareHeadingTune(false);
        prepareGoal(false);
    }
    document.getElementById("button_down").style.display = style;
    document.getElementById("button_up").style.display = style;
    document.getElementById("button_left").style.display = style;
    document.getElementById("button_right").style.display = style;
    document.getElementById("button_emstop").style.display = style;
    document.getElementById("button_shit").style.display = style;
}

function prepareCtrlTune(show) {
    var style = "none";
    if(show)
    {
        style = "block";
        prepareJoydrive(false);
        prepareHeadingTune(false);
        prepareGoal(false);
    }
    document.getElementById("input_dest").style.display = style;
    document.getElementById("output_dest").style.display = style;
    document.getElementById("button_startstep").style.display = style;
    document.getElementById("button_emstop2").style.display = style;
    document.getElementById("select_sequence").style.display = style;
    document.getElementById("input_memdepth").style.display = style;
    document.getElementById("output_memdepth").style.display = style;
    document.getElementById("select_current").style.display = style;
}

function prepareHeadingTune(show) {
    var style = "none";
    if(show)
    {
      style = "block";
      prepareJoydrive(false);
      prepareCtrlTune(false);
      prepareGoal(false);
    }

    document.getElementById("button_startstep").style.display = style;
    document.getElementById("input_memdepth").style.display = style;
    document.getElementById("output_memdepth").style.display = style;
}

function prepareGoal(show) {
    var style = "none";
    if(show)
    {
      style = "block";
      prepareJoydrive(false);
      prepareCtrlTune(false);
      prepareHeadingTune(false);
    }

    document.getElementById("input_heading").style.display = style;
    document.getElementById("output_heading").style.display = style;
    document.getElementById("input_distance").style.display = style;
    document.getElementById("output_distance").style.display = style;
    document.getElementById("button_startstep").style.display = style;
    document.getElementById("button_emstop2").style.display = style;
}

function select_topic_onchange() {
    var selected = document.getElementById("select_topic").value;
 
    visualizer.setCanvas();
    if(odomShow == selected)
    {
      topics = topicsManager.getAllTopicDescriptions();
      visualizer.clearCanvas();
      var subscribed = []
      for (i in topics) 
      {
        var current = topics[i];

        for (k in current) {
            var topic = current[k];
            if("/odom_wheel" == topic[1] || "/odom" == topic[1])
            {
              subscribed.push(topic[0])
            }
        }
      }
      topicsManager.subscribeTopics(subscribed);
      visualizer.enableOdometry(true);
      visualizer.enableMap(false);
      visualizer.enableTrack(false);
    }
    else if(mapShow == selected)
    {
      topics = topicsManager.getAllTopicDescriptions();
      visualizer.clearCanvas();
      var subscribed = []
      for (i in topics) 
      {
        var current = topics[i];

        for (k in current) {
            var topic = current[k];
            if("/map" == topic[1] || "/odom" == topic[1])
            {
              subscribed.push(topic[0])
            }
        }
      }
      topicsManager.subscribeTopics(subscribed);
      visualizer.enableOdometry(false);
      visualizer.enableMap(true);
      visualizer.enableTrack(false);
    }
    else if(trackShow == selected)
    {
      topics = topicsManager.getAllTopicDescriptions();
      visualizer.clearCanvas();
      var subscribed = []
      for (i in topics) 
      {
        var current = topics[i];

        for (k in current) {
            var topic = current[k];
            if("/route" == topic[1] ||/* "/gps_data"*/ "/ublox_gps/fix" == topic[1])
            {
              subscribed.push(topic[0])
            }
        }
      }
      topicsManager.subscribeTopics(subscribed);
      visualizer.enableOdometry(false);
      visualizer.enableMap(false);
      visualizer.enableTrack(true);
    }
    else
    {
      visualizer.enableOdometry(false);
      visualizer.enableMap(false);
      visualizer.enableTrack(false);
      topicsManager.subscribeTopic(selected);
      visualizer.clearCanvas();
      visualizer.showIdle();
    }
}

function select_topic_refresh() {
    topics = topicsManager.getAllTopicDescriptions();

    var $select = $('#select_topic');
    $select.find('optgroup').remove().end();
    $select.find('option').remove().end();

    var value = 0;

    for (i in topics) {
        var current = topics[i];
        var $optgroup = $("<optgroup label = '" + String(i) + "'>");

        for (k in current) {
            var topic = current[k];
            value = topic[0];
            var opt = "<option value=" + value + ">" + String(topic[1]) + "</option>";
            $optgroup.append(opt)
        }
        $optgroup.appendTo($select);
    }
    var $optgroup = $("<optgroup label = 'special'>");
    value = value + 1;
    var opt = "<option value=" + value + ">odometry compare</option>";
    odomShow = value;
    $optgroup.append(opt);
    value = value + 1;
    var opt = "<option value=" + value + ">map</option>";
    mapShow = value;
    $optgroup.append(opt);
    $optgroup.appendTo($select);
    value = value + 1;
    var opt = "<option value=" + value + ">track</option>";
    trackShow = value;
    $optgroup.append(opt);
    $optgroup.appendTo($select);
}

function select_control_onchange() {
    opMode = document.getElementById("select_control").value;
    switch(opMode)
    {
        case "joydrive": prepareJoydrive(true); break;
        case "ctrltune": prepareCtrlTune(true); break;
        case "headingtune": prepareHeadingTune(true); break;
        case "goal": prepareGoal(true); break;
        default: break;
    }
}

function input_dest_oninput()
{
    var value = document.getElementById("input_dest").value;
    document.getElementById("output_dest").innerHTML = "Step: " + value;
}

function button_startstep_onclick()
{
    var mode = document.getElementById("select_sequence").value;
    var step = document.getElementById("input_dest").value;
    var memdepth = document.getElementById("input_memdepth").value;
    if("ctrltune" == opMode)
    {
      var rpm = ("rpm" == document.getElementById("select_current").value);
      tuner.doSequence(mode, memdepth, step, rpm);
    }
    else if("headingtune" == opMode)
    {
      headingTuner.request(memdepth);
    }
    else if("goal" == opMode)
    {
      goalSetter.request(parseFloat(document.getElementById("input_distance").value),
      parseFloat(document.getElementById("input_heading").value));
    }
}

function input_memdepth_oninput()
{
    var value = document.getElementById("input_memdepth").value;
    document.getElementById("output_memdepth").innerHTML = "Mem: " + value;
}

function button_emstop2_onclick()
{
    tuner.reset();
    goalSetter.cancel();
}

function button_download_onclick()
{
    visualizer.download();
}
