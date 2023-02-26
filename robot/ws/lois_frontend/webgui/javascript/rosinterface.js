var wsUri = "ws://"+location.hostname+":9090";

var ros = new ROSLIB.Ros({
    url : wsUri
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');

});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

function topicsCallback(manager, result)
{
    manager.updateTopics(result);
}

function getTopics(manager) {
    var topicsClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

    topicsClient.callService(request, function(result) {
        topicsCallback(manager, result);
    });
};
