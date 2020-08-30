var mqtt = require('mqtt'); //https://www.npmjs.com/package/mqtt
var Topic = 'rainflow/data/#'; //subscribe to all topics
var Broker_URL = 'mqtt://test.mosquitto.org';
var Database_URL = '127.0.0.1';

var options = {
	clientId: 'rainflow_solar',
	port: 1883,
	//username: 'mqtt_user',
	//password: 'mqtt_password',
	keepalive : 60
};

var client  = mqtt.connect(Broker_URL, options);
client.on('connect', mqtt_connect);
client.on('reconnect', mqtt_reconnect);
client.on('error', mqtt_error);
client.on('message', mqtt_messsageReceived);
client.on('close', mqtt_close);

function mqtt_connect() {
    //console.log("Connecting MQTT");
    client.subscribe(Topic, mqtt_subscribe);
};

function mqtt_subscribe(err, granted) {
    console.log("Subscribed to " + Topic);
    if (err) {console.log(err);}
};

function mqtt_reconnect(err) {
    //console.log("Reconnect MQTT");
    //if (err) {console.log(err);}
	client  = mqtt.connect(Broker_URL, options);
};

function mqtt_error(err) {
    //console.log("Error!");
	//if (err) {console.log(err);}
};

function after_publish() {
	//do nothing
};

//receive a message from MQTT broker
function mqtt_messsageReceived(topic, message, packet) {
	var topic_str = topic.toString();
	var topic_str_buffer = topic_str.replace('rainflow/data/','');
	// console.log("Device ID: " + topic_str_buffer);
	// console.log(topic_str_buffer);
	var message_str = message.toString(); //convert byte array to string
	// message_str = message_str.replace(/\n$/, ''); //remove new line
	// payload syntax: clientID,topic,message
	// if (countInstances(message_str) != 1) {
		// console.log("Invalid payload");
		// } else {	
		// insert_message(topic, message_str, packet);
		// console.log(message_arr);
	// }
	insert_message(topic, message_str, packet);
};

function mqtt_close() {
	//console.log("Close MQTT");
};

////////////////////////////////////////////////////
///////////////////// MYSQL ////////////////////////
////////////////////////////////////////////////////
var mysql = require('mysql'); //https://www.npmjs.com/package/mysql
//Create Connection
var connection = mysql.createConnection({
	host: Database_URL,
	user: "mqtt_listener",
	password: "0pzrilTWtaMx1KCK",
	database: "rainflow_test"
});

connection.connect(function(err) {
	if (err) throw err;
	console.log("Database Connected!");
});

//insert a row into the tbl_messages table
function insert_message(topic, message_str, packet) {
	var topic_str = topic.toString();
	var topic_str_buffer = topic_str.replace('rainflow/data/','');

	console.log("");
	console.log("Device ID: " + topic_str_buffer);
	var message_arr = extract_string(message_str); //split a string into an array
	var data = JSON.parse(message_str.payloadString);


	var sql = "INSERT INTO ?? (??,??,??,??,??,??,??,??,??,??,??) VALUES (?,?,?,?,?,?,?,?,?,?,?)";
	var params = [topic_str_buffer, 
				'SP1CRaw', 'SP1CVoltage', 'SP1Current', 'SP1VRaw', 'SP1Voltage', 'SP1Power',  'SPRCRaw', 'SPRCVoltage', 'SPRCurrent', 'BVRaw', 'BVoltage',
				data.SP1CRaw, data.SP1CVoltage, data.SP1Current, data.SP1VRaw, data.SP1Voltage, data.SP1Power,  data.SPRCRaw, data.SPRCVoltage, data.SPRCurrent, data.BVRaw, data.BVoltage];
	sql = mysql.format(sql, params);

	connection.query(sql, function (error, results) {
		if (error) throw error;
		console.log("Published: " + message_str);
	}); 
};	

//split a string into an array of substrings
function extract_string(message_str) {
	var message_arr = message_str.split(","); //convert to array	
	return message_arr;
};	

//count number of delimiters in a string
var delimiter = ",";
function countInstances(message_str) {
	var substrings = message_str.split(delimiter);
	return substrings.length - 1;
};