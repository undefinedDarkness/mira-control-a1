package main

import (
	"log"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
)

type Master struct {
	pixhawk *gomavlib.Node
	ros *goroslib.Node
}

func (app *Master) onCommandMessage(msg *Commands) {

}

func (app *Master) onROVMessage(msg *Commands) {

}

func (app *Master) waitForHeartbeat() {}

func main() {
	// -- INIT NODE --
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name: "pymav_master",
	});

	if err != nil {
		log.Fatal("Failed to register node!, Is roscore running?\n", err);
	} 
	defer n.Close();

	// MAVLINK
	mav_node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{
				Device: "/dev/Pixhawk",
				Baud: 115200,
			},
		},
		Dialect: ardupilotmega.Dialect,
		OutVersion: gomavlib.V2,
	});

	if err != nil {
		log.Fatal("Failed to connect to Pixhawk!\n", err);
	}
	defer mav_node.Close();

	// Setup State
	app := Master{
		ros: n,
		pixhawk: mav_node,
	}

	// -- INITIALIZE SUBSCRIBERS AND PUBLISHERS --
	command_sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node: n,
		Topic: "/master/commands",
		Callback: app.onCommandMessage,
		QueueSize: 1,
	});

	if err != nil {
		log.Fatal("Failed to register /master/commands subscriber\n", err);
	}
	defer command_sub.Close();

	thruster_sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node: n,
		Topic: "/rov/commands",
		QueueSize: 1,
		Callback: app.onROVMessage,
	});
	if err != nil {
		log.Fatal("Failed to register /rov/commands subscriber\n", err);
	}
	defer thruster_sub.Close();

	telemetry_pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node: n,
		Topic: "/master/telemetry",
		Msg: Telemetry{},
	})

	if err != nil {
		log.Fatal("Failed to register /master/telemetry publisher\n", err);
	}
	defer telemetry_pub.Close();
}
