class Topic
{
    constructor(name, type, visualizer)
    {
        this.name = name;
        this.type = type;
        this.subscribed = false;
        this.visualizer = visualizer;

        this.listener = new ROSLIB.Topic({
            ros : ros,
            name : this.name,
            messageType : this.type
        });
    }

    getName()
    {
        return this.name;
    }
    
    getType()
    {
        return this.type;
    }

    subscribe()
    {
        var name = this.name;
        var type = this.type;
        this.subscribed = true;
        var vis = this.visualizer;
        this.listener.subscribe(function(message) {
            vis.show(name, type, message);
        });
    }

    unsubscribe()
    {
        this.subscribed = false;
        this.listener.unsubscribe();
    }
}

class TopicsManager
{
    constructor(visualizer, update){
        this.topics = [];
        this.currentTopic = 0;
        this.visualizer = visualizer;
        this.update = update;
    }

    updateTopics(nameAndTypeList2d)
    {
        this.topics = [];
        for(var i in nameAndTypeList2d.topics)
        {
            this.topics[i] = new Topic(nameAndTypeList2d.topics[i], nameAndTypeList2d.types[i], this.visualizer);
        }
        if(this.update)
        {
          select_topic_refresh();
        }
    }

    getAllTopicDescriptions()
    {
        var descriptions = [];
        for(var i in this.topics)
        {
            var optGroup = String(this.topics[i].getType());

            if(descriptions[optGroup] === undefined)
            {
                descriptions[optGroup] = [];
            }

            var name = String(this.topics[i].getName())
            var curr = [parseInt(i), name];
            descriptions[optGroup].push(curr);
        }
        return descriptions;
    }

    unsubscribeAll()
    {
        var t = 0;
        for(t in this.topics)
        {
          this.topics[t].unsubscribe();
        }
    }

    subscribeTopic(listIndex)
    {
        this.unsubscribeAll();
        this.currentTopic = listIndex;
        this.topics[listIndex].subscribe();
    }

    getName(listIndex)
    {
      return this.topics[listIndex].getName();
    }

    subscribeTopics(listIndices)
    {
      this.unsubscribeAll();
      for(i in listIndices)
      {
        this.currentTopic = listIndices[i];
        this.topics[listIndices[i]].subscribe();
      }
    }
}

