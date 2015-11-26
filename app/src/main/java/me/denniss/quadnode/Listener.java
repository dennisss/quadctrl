package me.denniss.quadnode;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Twist;
import std_msgs.*;
import std_msgs.String;


/*
* Listens for movement instructions from a controller
*/
public class Listener extends AbstractNodeMain {
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("quadnode/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

        Subscriber<Twist> sub = connectedNode.newSubscriber("pose", Twist._TYPE);
        sub.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist twist) {

                // Tell the NDK to do something
                Quadcopter.setthrottle((float)twist.getLinear().getZ());

            }
        });


        final Log log = connectedNode.getLog();
        Subscriber<String> subscriber = connectedNode.newSubscriber("chatter", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                log.info("I heard: \"" + message.getData() + "\"");
            }
        });
    }

}
