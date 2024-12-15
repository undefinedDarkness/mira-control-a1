package main

import (
	"bufio"
	"math"
	"os"
	"reflect"
	"strings"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/bluenviron/goroslib/v2"
	"github.com/charmbracelet/log"
)

type Master struct {
	// Connections
	pixhawk *gomavlib.Node
	ros     *goroslib.Node
	target_system uint8
	target_component uint8
	// Publishers & Subscribers
	command_sub   *goroslib.Subscriber
	thruster_sub  *goroslib.Subscriber
	telemetry_pub *goroslib.Publisher
	// Channels
	heartbeat_ch chan struct{}
	ack_ch       chan ardupilotmega.MessageCommandAck
	telemetry_ch chan message.Message
	// State
	autonomous  bool
	channel_arr [8]int
	armed       bool
	mode        string
	// Misc
	log_misc bool
}

func NewApplication(_pixhawk *gomavlib.Node, _ros *goroslib.Node) Master {
	log.Info("[*] Initializing new application")
	app := Master{
		pixhawk: _pixhawk,
		ros:     _ros,
		target_system: 0,
		target_component: 0,

		heartbeat_ch: make(chan struct{}),
		ack_ch:       make(chan ardupilotmega.MessageCommandAck),
		telemetry_ch: make(chan message.Message),

		autonomous: false,
		log_misc: true,
	}
	n := app.ros

	// -- INITIALIZE SUBSCRIBERS AND PUBLISHERS --
	command_sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:      n,
		Topic:     "/master/commands",
		Callback:  app.onCommandMessage,
		QueueSize: 1,
	})
	if err != nil {
		log.Fatal("[ROS] Failed to register /master/commands subscriber\n", err)
	}
	app.command_sub = command_sub

	thruster_sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:      n,
		Topic:     "/rov/commands",
		QueueSize: 1,
		Callback:  app.onROVMessage,
	})

	if err != nil {
		log.Fatal("[ROS] Failed to register /rov/commands subscriber\n", err)
	}
	app.thruster_sub = thruster_sub

	telemetry_pub, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "/master/telemetry",
		Msg:   &Telemetry{},
	})

	if err != nil {
		log.Fatal("[ROS] Failed to register /master/telemetry publisher\n", err)
	}
	app.telemetry_pub = telemetry_pub

	return app
}

func (app *Master) cleanup() {
	log.Info("[*] Cleanup up\n")
	app.ArmOrDisarm(false)
	app.telemetry_pub.Close()
	app.command_sub.Close()
	app.thruster_sub.Close()
	app.ros.Close()
	app.pixhawk.Close()
}

func (app *Master) ArmOrDisarm(arm bool) {
	app.armed = arm
	<-app.heartbeat_ch
	var arm_v float32 = 0.0
	if arm {
		arm_v = 1.0
	}

	var confirmation_count uint8 = 0
	var ack_result common.MAV_RESULT = common.MAV_RESULT_UNSUPPORTED;
	ack_loop:
	for {
		err := app.pixhawk.WriteMessageAll(&ardupilotmega.MessageCommandLong{
			TargetSystem: app.target_system,
			TargetComponent: app.target_component,
			Command: common.MAV_CMD(ardupilotmega.MAV_CMD_COMPONENT_ARM_DISARM),
			Confirmation: confirmation_count,
			Param1: arm_v,
			Param2: 21196,
		});

		if err != nil {
			log.Error("Error sending command to pixhawk", err);
		}

		select {
		case <-time.After(2 * time.Second):
			log.Info("Ack timed out");
			break;
		case ack := <-app.ack_ch:
			if (ack.Command == common.MAV_CMD(ardupilotmega.MAV_CMD_COMPONENT_ARM_DISARM)) {
				log.Info("Got response from pixhawk, ", "ack", ack);
				ack_result = ack.Result;
				break ack_loop;
			} else {
				log.Info("Got wrong ack", "ack", ack);
				break ack_loop;
			} 
		}

		confirmation_count = confirmation_count + 1;
		if (confirmation_count >= 5) {
			log.Error("Failed to ARM / DISARM pixhawk, no ack after 255 commands");
			break;
		}
	}

	if arm {
		log.Warn("[PIXHAWK] ARM Command Sent", "result", ack_result);
	} else {
		log.Warn("[PIXHAWK] DISARM Command Sent", "result", ack_result);
	}


}

func (app *Master) SwitchMode(mode string) {

	var values_SUB_MODE = map[string]ardupilotmega.SUB_MODE{
		"STABILIZE": ardupilotmega.SUB_MODE_STABILIZE,
		"ACRO":      ardupilotmega.SUB_MODE_ACRO,
		"ALT_HOLD":  ardupilotmega.SUB_MODE_ALT_HOLD,
		"AUTO":      ardupilotmega.SUB_MODE_AUTO,
		"GUIDED":    ardupilotmega.SUB_MODE_GUIDED,
		"CIRCLE":    ardupilotmega.SUB_MODE_CIRCLE,
		"SURFACE":   ardupilotmega.SUB_MODE_SURFACE,
		"POSHOLD":   ardupilotmega.SUB_MODE_POSHOLD,
		"MANUAL":    ardupilotmega.SUB_MODE_MANUAL,
	}

	mode = strings.ToUpper(mode)

	mode_val, ok := values_SUB_MODE[mode]

	if !ok {
		log.Warn("Invalid Mode: ", "mode", mode)
		return
	}

	app.mode = mode
	err := app.pixhawk.WriteMessageAll(&ardupilotmega.MessageSetMode{
		BaseMode:   common.MAV_MODE(common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
		CustomMode: uint32(mode_val),
	})

	if err != nil {
		log.Error("Failed to set pixhawk mode", "error", err);
	}

}

func (app *Master) onCommandMessage(msg *Commands) {
	if !app.autonomous {
		return
	}

	app.evaluateCommand(msg)
}

func (app *Master) evaluateCommand(msg *Commands) {

	app.channel_arr = [8]int{
		int(msg.Pitch),
		int(msg.Roll),
		int(msg.Thrust),
		int(msg.Yaw),
		int(msg.Forward),
		int(msg.Lateral),
		int(msg.Servo1),
		int(msg.Servo2),
	}

	if app.armed != msg.Arm {
		app.ArmOrDisarm(msg.Arm)
	}

	if app.mode != msg.Mode {
		if app.armed {
			log.Warn("Disarm Pixhawk to change modes")
			return
		} else {
			app.SwitchMode(msg.Mode)
		}
	}
}

func (app *Master) onROVMessage(msg *Commands) {
	if app.autonomous {
		return
	}

	app.evaluateCommand(msg)
}

// TODO: Autogenerate from ardupilotmega.xml
const (
	SYS_STATUS_MSG_ID          = 1
	HEARTBEAT_MSG_ID           = 0
	ATTITUDE_QUATERNION_MSG_ID = 31
	AHRS2_MSG_ID               = 178
	SCALED_PRESSURE2_MSG_ID    = 137
	VFR_HUD_MSG_ID             = 74
	SCALED_IMU2_MSG_ID         = 116
)

func (app *Master) handleTelemetry() {
	telemetry_msg := Telemetry{}

	go app.requestMessageAtInterval(SYS_STATUS_MSG_ID, 100)
	go app.requestMessageAtInterval(HEARTBEAT_MSG_ID, 100)
	go app.requestMessageAtInterval(ATTITUDE_QUATERNION_MSG_ID, 100)
	go app.requestMessageAtInterval(AHRS2_MSG_ID, 100)
	go app.requestMessageAtInterval(SCALED_IMU2_MSG_ID, 100)
	go app.requestMessageAtInterval(VFR_HUD_MSG_ID, 100)
	go app.requestMessageAtInterval(SCALED_PRESSURE2_MSG_ID, 100)

	for {
		data := <-app.telemetry_ch
		switch msg := data.(type) {
		case *ardupilotmega.MessageSysStatus:
			telemetry_msg.BatteryVoltage = float32(msg.VoltageBattery) / 1000

			if telemetry_msg.BatteryVoltage < 15 {
				log.Warn("Battery Critically Low ", "battery level", telemetry_msg.BatteryVoltage)
			}
		case *ardupilotmega.MessageScaledImu:
			telemetry_msg.Timestamp = int32(msg.TimeBootMs)
			telemetry_msg.ImuGyroX = int32(msg.Xgyro)
			telemetry_msg.ImuGyroY = int32(msg.Ygyro)
			telemetry_msg.ImuGyroZ = int32(msg.Zgyro)
			telemetry_msg.ImuGyroCompassX = int32(msg.Xmag)
			telemetry_msg.ImuGyroCompassY = int32(msg.Ymag)
		case *ardupilotmega.MessageAttitudeQuaternion:
			telemetry_msg.Q1 = msg.Q1
			telemetry_msg.Q2 = msg.Q2
			telemetry_msg.Q3 = msg.Q3
			telemetry_msg.Q4 = msg.Q4
			telemetry_msg.Rollspeed = msg.Rollspeed
			telemetry_msg.Pitchspeed = msg.Pitchspeed
			telemetry_msg.Yawspeed = msg.Yawspeed
		case *ardupilotmega.MessageVfrHud:
			telemetry_msg.Heading = int32(msg.Heading)
			telemetry_msg.InternalPressure = msg.Alt
		case *ardupilotmega.MessageScaledPressure2:
			telemetry_msg.ExternalPressure = msg.PressAbs
		default:
			log.Warn("Unhandled telemetry message recieved ", "msg", reflect.TypeOf(data).String());
			continue
		}
		app.telemetry_pub.Write(&telemetry_msg)
	}
}

// Modify SINGLE RC channel PWM
func (app *Master) SetRCChannelPWM(channel int, pwm int) {
	if channel < 1 || channel > 8 {
		log.Warn("Invalid Channel ", "channel", channel)
		return
	}

	rc_channels := [8]int{}
	for i := range rc_channels {
		rc_channels[i] = math.MaxUint16
	}

	rc_channels[channel-1] = pwm // BAD: 1 indexing
	app.pixhawk.WriteMessageAll(&ardupilotmega.MessageRcChannelsOverride{
		TargetSystem:    0,
		TargetComponent: 0,
		Chan1Raw:        uint16(rc_channels[0]),
		Chan2Raw:        uint16(rc_channels[1]),
		Chan3Raw:        uint16(rc_channels[2]),
		Chan4Raw:        uint16(rc_channels[3]),
		Chan5Raw:        uint16(rc_channels[4]),
		Chan6Raw:        uint16(rc_channels[5]),
		Chan7Raw:        uint16(rc_channels[6]),
		Chan8Raw:        uint16(rc_channels[7]),
		Chan9Raw:        math.MaxUint16,
		Chan10Raw:       math.MaxUint16,
		Chan11Raw:       math.MaxUint16,
		Chan12Raw:       math.MaxUint16,
		Chan13Raw:       math.MaxUint16,
		Chan14Raw:       math.MaxUint16,
		Chan15Raw:       math.MaxUint16,
		Chan16Raw:       math.MaxUint16,
	})
}

// TODO: Move to single RC command, instead of 8???
func (app *Master) Actuate() {
	for i, pwm := range app.channel_arr {
		app.SetRCChannelPWM(i+1, pwm)
	}
}

func (app *Master) updateRCChannels() {
	for {
		app.Actuate()
	}
}

func (app *Master) handleEventFrame(frm *gomavlib.EventFrame) {
	// TODO: Find better solution
	app.target_component = frm.ComponentID();
	app.target_system = frm.SystemID();
	switch msg := frm.Message().(type) {
	case *ardupilotmega.MessageHeartbeat:
		log.Info("[PIXHAWK] Recieved heartbeat");
		app.heartbeat_ch <- struct{}{}
	case *common.MessageCommandAck:
		log.Info("[PIXHAWK] Recieved Command Acknowledgement")
		app.ack_ch <- *msg
	case *ardupilotmega.MessageStatustext:
		log.Info("[PIXHAWK]", "status_text", msg.Text);
	case *ardupilotmega.MessageAhrs2, *ardupilotmega.MessageScaledImu2, *ardupilotmega.MessageVfrHud, *ardupilotmega.MessageAttitudeQuaternion, *ardupilotmega.MessageScaledPressure2, *ardupilotmega.MessageSysStatus:
		if app.log_misc {
			log.Info("[PIXHAWK] Recieved Telemetry Message", "message", reflect.TypeOf(msg).String());
		}
		app.telemetry_ch <- msg
	default:	
		if app.log_misc {
			log.Warn("[PIXHAWK] Unhandeled Message Type ", "msg_type", reflect.TypeOf(msg).String(), "msg", msg);
		}
		return;
	}
}

func (app *Master) listenForEvents() {
// loop:
	for evt := range app.pixhawk.Events() {
		switch v := evt.(type) {
		case *gomavlib.EventChannelClose:
			log.Info("[PIXHAWK] Event channel closed")
			// break loop
		case *gomavlib.EventChannelOpen:
			log.Info("[PIXHAWK] Event channel opened")
		case *gomavlib.EventParseError:
			log.Warn("[PIXHAWK] Event parse error ", "err", v)
		case *gomavlib.EventFrame:
			go app.handleEventFrame(v)
		}
	}
}

func (app *Master) waitForHeartbeat() {
	<-app.heartbeat_ch
}

func (app *Master) requestMessageAtInterval(msgId uint16, freq_hz int) {
	log.Info("Request message at interval", "msgid", msgId, "freq", freq_hz);
	app.pixhawk.WriteMessageAll(&ardupilotmega.MessageMessageInterval{
		IntervalUs: int32(1e6 / freq_hz),
		MessageId:  msgId,
	})

	// need to wait till its own acknowledgement comes through
	ack := <-app.ack_ch
	if ack.Result == ardupilotmega.MAV_RESULT_ACCEPTED && ack.Command == common.MAV_CMD_SET_MESSAGE_INTERVAL {
		log.Info("[PIXHAWK] Succeded request to modify interval of message", "id", msgId)
	} else {
		log.Error("[PIXHAWK] Failed request to modify interval of message", "id", msgId)
	}
}

func main() {
	// -- INIT NODE --
	log.Info("[ROS] Connecting to ROS Core")
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name: "pymav_master",
	})

	if err != nil {
		log.Fatal("[ROS] Failed to register node!, Is roscore running?", "err", err)
	}
	defer n.Close()

	// MAVLINK
	log.Info("[PIXHAWK] Connecting to PixHawk\n")
	mav_node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{
				Device: "/dev/Pixhawk",
				Baud:   115200,
			},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2,
		OutSystemID: 11,
		HeartbeatDisable: true,
	})

	if err != nil {
		log.Fatal("[PIXHAWK] Failed to connect to Pixhawk!", err)
	}
	defer mav_node.Close()

	// Setup State
	app := NewApplication(mav_node, n)

	var wg sync.WaitGroup;
	defer wg.Wait();

	wg.Add(4);
	go func() {
		app.listenForEvents();
		log.Warn("[*] Finished listening for events");
		wg.Done();
	}(); 

	go func(){
		app.handleTelemetry();
		log.Warn("[*] Finished handling telemetry");
		wg.Done();
	}();

	go func(){
		app.updateRCChannels();
		log.Warn("[*] Finished updating RC channels");
		wg.Done();
	}();

	// Start listening for user input to arm or disarm the vehicle
	go func() {
		scanner := bufio.NewScanner(os.Stdin)
		for {
			scanner.Scan() // Capture user input
			input := scanner.Text()

			switch input {
			case "arm":
				app.ArmOrDisarm(true)
			case "disarm":
				app.ArmOrDisarm(false)
			default:
				log.Warn("Invalid input, please enter 'arm' or 'disarm'.")
			}
		}
	}()

	app.waitForHeartbeat();
	log.Info("[*] Init Complete\n")
}
