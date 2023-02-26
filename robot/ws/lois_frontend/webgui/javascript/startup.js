function autoscale()
{ 
  scale(2000, 1000)
};

function autoscale2()
{ 
  scale(3000, 2000)
};

function scale(facX, facY)
{
    ratioW = window.innerWidth / facX;
    ratioH = window.innerHeight / facY;
    ratio = 2 * Math.min(ratioH, ratioW);
    $('body').css('-moz-transform', 'scale(' + ratio + ')');
    $('body').css('-o-transform', 'scale(' + ratio + ')');
    $('body').css('-webkit-transform', 'scale(' + ratio + ')');
    $('body').css('transform', 'scale(' + ratio + ')');
    console.log("autoscale")
}

var visualizer = new Visualizer("div_screen", "canvas_screen");
var topicsManager = new TopicsManager(visualizer, true); 
function init()
{
//    autoscale();
    getTopics(topicsManager);
    window.onresize = autoscale;
    document.addEventListener('DOMContentLoaded', init, false);	
    prepareJoydrive(true);
}

function initDash()
{
          var vis1 = new Visualizer("div_road", "canvas_road");
          var topicsManager1 = new TopicsManager(vis1, false);
          var vis2 = new Visualizer("div_filter", "canvas_filter");
          var topicsManager2 = new TopicsManager(vis2, false);
          var vis3 = new Visualizer("div_global", "canvas_global");
          var topicsManager3 = new TopicsManager(vis3, false);
          var vis4 = new Visualizer("div_local", "canvas_local");
          var topicsManager4 = new TopicsManager(vis4, false);
          getTopics(topicsManager1);
          getTopics(topicsManager2);
          getTopics(topicsManager3);
          getTopics(topicsManager4);
         
          var subscribed1 = [];
          var subscribed2 = [];
          var subscribed3 = [];
          var subscribed4 = [];
          
          function init(){setTimeout(function(){
          var topics = topicsManager1.getAllTopicDescriptions();
          for (i in topics) {
                    var current = topics[i];
                    for (k in current) {
                      var topic = current[k];
                      console.log(topic);
                      if("/road_detector/roaddetector/way/compressed" == topic[1])
                      {
                        subscribed1.push(topic[0]);
                      }
                      if("/road_detector/roaddetector/filter/compressed" == topic[1])
                      {
                        subscribed2.push(topic[0]);
                      }
//                      if("/road_detector/road_lidar" == topic[1])
                      if("/grid_planner/planner/out/compressed" == topic[1])
                      {
                        subscribed4.push(topic[0]);
                      }
                      if(/*"/gps_data"*/"/ublox_gps/fix" == topic[1])
                      {
                        subscribed3.push(topic[0]);
                      }
                      if("/route" == topic[1])
//                      if("/road_detector/roaddetector/out/compressed" == topic[1])
                      {
                        subscribed3.push(topic[0]);
                      }
                   }
                   }
                   topicsManager1.subscribeTopics(subscribed1);
                   topicsManager2.subscribeTopics(subscribed2);
                   topicsManager3.subscribeTopics(subscribed3);
                   topicsManager4.subscribeTopics(subscribed4);
                   vis3.enableTrack(true);
        window.onresize = autoscale2 }, 2000)}
        document.addEventListener('DOMContentLoaded', init, false);
}
