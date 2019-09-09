#!/usr/bin/bash
# -*- coding: utf-8 -*

from flask import Flask, render_template, jsonify, url_for
from flask import Markup
from flask_mqtt import Mqtt

from threading import Thread

from getcaller import get_devices

import constants

from ncclient import manager
from ncclient import xml_ 

from lxml import etree		#needed because ncclient sends lxml, not xml object

import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement, tostring

import threading

import logging 	#not really needed but silences ajax logs (occurring once per second per client) 
				#see variable suppress_werkzeugs_logs 

from socket import error as socket_error



app = Flask(__name__)

html_template_file = 'interface.html'

target_ip = '127.0.0.1'
#target_ip = '192.168.178.80'
target_ip = 'localhost'
#target_ip = '141.89.175.111'
target_port = 44555
target_username = 'user'
target_password = 'admin'

app.config['MQTT_BROKER_URL'] = target_ip
app.config['MQTT_BROKER_PORT'] = 1883
# for auth, config is ['MQTT_USERNAME'] and ['MQTT_PASSWORD']
app.config['MQTT_REFRESH_TIME'] = 1.0 


device_dict={}
sensor_dict={}

script_root=''

script_error=""
script_notification=""
script_notification_old="will get overwritten after first notification"
script_notification_counter=0



suppress_werkzeugs_logs = True

#silencing werkzeug logs
if(suppress_werkzeugs_logs):
	log = logging.getLogger('werkzeug')
	log.setLevel(logging.ERROR)
	
	
#gets called when user visits website, also used for "get devices"-button click
@app.route("/", methods=['POST','GET'])
def root():
	global script_root #needed for ajax.js
	script_root = url_for('root', _external=True)
	clear_data()
	
	if ('m' in globals() and m.connected): 	#checks for existing manager (NETCONF connection) to get devices
		get_device_dict()
	else:					#(re)connects if not connected to NETCONF Server 
		async_establish_manager_connection()
		
	return render_template(html_template_file, things_list=list_things(), errors=script_error);


#ajax used for errors, notifications, and providing sensor data
@app.route('/ajax', methods= ['GET'])
def ajax():
    global script_notification
    global script_notification_old
    global script_notification_counter
    
    #used for making notification disappear after a while
    if(script_notification_old==script_notification):
    	script_notification_counter+=1
    if(script_notification_counter>8):
    	script_notification=""
    	script_notification_counter=0
    
    script_notification_old=script_notification
    
    return jsonify(error=script_error, notification=script_notification, sensors=sensor_dict)


#user clicked an rpc button
@app.route('/function_call/<thing>/<function>',defaults={'param_name': None, 'value': None},methods=['GET', 'POST'])
@app.route('/function_call/<thing>/<function>/<param_name>/<value>',methods=['GET', 'POST'])
def function_click(thing, function, param_name, value):
	clear_ajax()

	n = xml_.to_ele('<'+function+'/>')			#NETCONF expects data in form of XML
	child = etree.SubElement(n, "uuidInput")
	child.text = thing
	
	if(value!=None):							#if parameter value, add xml-node
		child = etree.SubElement(n, param_name)
		child.text = value
	else:							
		value = ""
				
	print("Request:")
	print(xml_.to_xml(n,pretty_print=True))
	
	t = Thread(target=rpc_call, args=(n,)) 	#async because it can take a while 
	t.start()								#and we don't want to block the main thread

	return render_template(html_template_file, things_list=list_things(),
	selected_rpc_text= Markup(constants.make_rpc_info(thing,function,value)), errors=script_error)


#generating and sending the actual call
def rpc_call(n):
	global script_notification
	try:
		rpc_reply = m.dispatch(n) 							#dispatches the request
		print(str(rpc_reply))
		rpc_reply_xml_root = ET.fromstring(str(rpc_reply))	#gets reply to put it in notification
		for data_node in rpc_reply_xml_root.findall('./data'):
			for retval_node in data_node.findall('./retval'):
				print(retval_node.text)
				script_notification = constants.rpc_notification_text(retval_node.text)
	except Exception as e:
		set_script_error("request error, connection still active? Netconf still running? Log: "+str(e))
		print(script_error)


def get_device_dict():
	global m
	global device_dict
	
	try:	
		device_dict = get_devices(m) #imported from namespaceparser.py
		print(device_dict)
	except (NameError, Exception ) as ee:
		if(ee.__class__.__name__=="NameError"):
			set_script_error('exception in get_devices(). '
			+ 'Probably manager not connected. '+'Log: '+str(ee))
		else:
			set_script_error('exception in get_devices(). Log: '+str(ee))
		print(script_error)


#iterates through device_dict, creates HTML-UI based on the contents of it + subscribes to mqtt
def list_things(): 
	content=''
	if(device_dict.items()):
		for device_uuid, dict_value in device_dict.items():
			
			#thing's name and ID as header for each thing "section"
			content = content + constants.make_thing_text(dict_value[0]);
			content = content + constants.make_thing_id_text(device_uuid);
			
			#RPCs, are going to be buttons
			functions = dict_value[1]['rpcs']
			if (functions):								#key: 	rpc name (displayed+called)
				for key, value in functions.items(): 	#value: array of: 
															#0 description (displayed on hover)
															#1 possible parameter leaf name
															#2 possible parameters to pass
					
					if(value[1]): #if RPC parameters contains parameters to pass
						content = content + constants.make_parameter_button(device_uuid, key, value[0], value[1], value[2])
					else:
						content = content + constants.make_function_button(device_uuid, key, value[0])
			
			#sensor data, going to be text fields updated by ajax.js
			sensors = dict_value[1]['sensors']
			if (sensors):	
				content = content + constants.make_linebreak()
				for key, value in sensors.items():
					content = content + constants.make_sensor_tf(key, value+device_uuid)
					if(mqtt):
						mqtt.subscribe(value+device_uuid)
					else:
						setup_mqtt()
						mqtt.subscribe(value+device_uuid)
		
	else:
		content = constants.make_linebreak() \
		+ constants.make_thing_id_text("No devices available, press \"Get Devices\" to refresh.")
			
	return Markup(content);


def async_establish_manager_connection():
	threading.Thread(target=establish_manager_connection).start()

#connects to NETCONF server via manager
def establish_manager_connection():
	global m
	try:
		m = manager.connect_ssh(target_ip, port=target_port, username=target_username,
		password=target_password,allow_agent=False,hostkey_verify=False,look_for_keys=False)
		
		get_device_dict()
		
	except socket_error as serr:
		set_script_error('socket_error exception in establish_manager_connection(). '
		+ 'Maybe unable to open socket. '+'Log: '+str(serr))
		print("establish_manager_connection(): Error socket_error couldnt open")
		print(serr)
	except Exception as e:
		set_script_error('exception in establish_manager_connection(). '+'Log: '+str(e))
		print("establish_manager_connection():  Exception something else not caught by socket_error")
		print(e)
		
#mqtt for receiving sensor values
def setup_mqtt():
	global mqtt
	mqtt = Mqtt(app)
	@mqtt.on_message()
	def handle_mqtt_message(client, userdata, message):
		print(message.topic)
		sensor_dict[message.topic]=(message.payload.decode(), constants.get_timestamp())
		#sensor_dict contains sensor values for each subscribed topic, gets queried at '/ajax' route


def set_script_error(error_string):
	global script_error
	script_error = error_string

		
def clear_data():
	clear_device_dict()
	clear_ajax()
	
def clear_device_dict():
	global device_dict
	device_dict={}	
	
def clear_ajax():
	global script_error 
	global script_notification 
	global script_notification_counter

	script_error=''
	script_notification=''
	script_notification_counter=0
	

if __name__ == "__main__":
	threading.Thread(target=setup_mqtt).start()	#MQTT client for sensor values
	async_establish_manager_connection()		#manager connection for scheme and RPCs
	app.run(debug=True, host='0.0.0.0')			#flask webserver launch







