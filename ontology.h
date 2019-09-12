#ifndef ONTOLOGY_H_
#define ONTOLOGY_H_
#include "project-conf.h"

const char * ontology = "[ {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Command\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ControllingFunctionality\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Device\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#MeasuringFunctionality\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Operation\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OperationInput\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OperationOutput\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OperationState\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OutputDataPoint\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Service\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ThingProperty\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesCommand\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesFunctionality\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#AnnotationProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasCommand\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasDataRestriction_pattern\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#DatatypeProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasFunctionality\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#AnnotationProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ],  \"http://www.w3.org/2000/01/rdf-schema#range\" : [ {    \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ThingProperty\"  } ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperation\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperationState\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOutput\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOutputDataPoint\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasService\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasSubService\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#ObjectProperty\" ]}, {  \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#DatatypeProperty\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Ontology\" ],  \"http://www.w3.org/2002/07/owl#imports\" : [ {    \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology-v0_9_0\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#YangDescription\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#Class\" ],  \"http://www.w3.org/2000/01/rdf-schema#subClassOf\" : [ {    \"@id\" : \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ThingProperty\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#cmdGreen\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Command\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#cmdOff\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Command\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#cmdOn\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Command\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#cmdSetColor\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Command\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#colorInput\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#colorInput\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OperationInput\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasDataRestriction_pattern\" : [ {    \"@value\" : \"all\"  }, {    \"@value\" : \"green\"  }, {    \"@value\" : \"orange\"  }, {    \"@value\" : \"red\"  }, {    \"@value\" : \"yellow\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#colorYangDesc\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#colorYangDesc\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"color parameter\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#deviceCategory\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ThingProperty\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"LED-LAMP\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#deviceDesc\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"MQTT-Device identified by UUID\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#deviceUuid\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ThingProperty\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"F97DF79-8A12-4F4F-8F69-6B8F3C2E78DD\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescBright\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"request brightness value\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescColor\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"set arbitrary LED color\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescGreen\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"Set the led color to green\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescSelectButton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"request button state\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescSwitchOff\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"Switches the led off\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescSwitchOn\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"Switches the led on\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescTemp\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"request temperature value\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcDescVoltage\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"request voltage value\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcGetBright\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#MeasuringFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescBright\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcGetSelButton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#MeasuringFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescSelectButton\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcGetSelectButton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcGetTemp\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#MeasuringFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescTemp\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcGetVoltage\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#MeasuringFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescVoltage\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcSetColor\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ControllingFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdSetColor\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescColor\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcSetGreen\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ControllingFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdGreen\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescGreen\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcSwitchOff\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ControllingFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdOff\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescSwitchOff\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#funcSwitchOn\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#ControllingFunctionality\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdOn\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcDescSwitchOn\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#mqttMethod\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#DatatypeProperty\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt#mqttTopic\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#DatatypeProperty\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt#myDevice\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Device\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasFunctionality\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcGetBright\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcGetSelButton\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcGetTemp\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcGetVoltage\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSetColor\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSetGreen\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSwitchOff\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSwitchOn\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasService\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#servNetconf\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#deviceCategory\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#deviceDesc\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#deviceUuid\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#opDescState\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasDataRestriction_pattern\" : [ {    \"@value\" : \"error\"  }, {    \"@value\" : \"nothing to do\"  }, {    \"@value\" : \"successful\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#opMqttGreen\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Operation\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdGreen\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperationState\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#opState\"  } ],  \"http://yang-netconf-mqtt#mqttMethod\" : [ {    \"@value\" : \"GREEN\"  } ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"led\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#opMqttOff\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Operation\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdOff\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperationState\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#opState\"  } ],  \"http://yang-netconf-mqtt#mqttMethod\" : [ {    \"@value\" : \"OFF\"  } ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"led\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#opMqttOn\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Operation\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdOn\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperationState\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#opState\"  } ],  \"http://yang-netconf-mqtt#mqttMethod\" : [ {    \"@value\" : \"ON\"  } ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"led\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#opMqttSetColor\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Operation\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesCommand\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#cmdSetColor\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#colorInput\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#uuidInput\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperationState\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#opState\"  } ],  \"http://yang-netconf-mqtt#mqttMethod\" : [ {    \"@value\" : \"color\"  } ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"led\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#opState\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OperationState\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasDataRestriction_pattern\" : [ {    \"@value\" : \"ERROR\"  }, {    \"@value\" : \"NOOP\"  }, {    \"@value\" : \"OK\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#opDescState\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#outDPVoltage\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt#outDpBrightness\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OutputDataPoint\" ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"sensor/brightness/als01/\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#outDpSelectButton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OutputDataPoint\" ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"sensor/button/select/\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#outDpTemperature\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OutputDataPoint\" ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"sensor/temperature/temp01/\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#outDpVoltage\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OutputDataPoint\" ],  \"http://yang-netconf-mqtt#mqttTopic\" : [ {    \"@value\" : \"sensor/voltage/vdd01/\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#paramYangDesc\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"parameter value\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servBrightness\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Service\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesFunctionality\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcGetBright\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOutputDataPoint\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#outDpBrightness\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servButton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servNetconf\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Service\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesFunctionality\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcGetBright\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcGetSelButton\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcGetTemp\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcGetVoltage\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSetColor\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSetGreen\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSwitchOff\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#funcSwitchOn\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOperation\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#opMqttGreen\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#opMqttOff\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#opMqttOn\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#opMqttSetColor\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasSubService\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#servBrightness\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#servSelectButton\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#servTemperature\"  }, {    \"@id\" : \"http://yang-netconf-mqtt#servVDD\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servSelectButton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Service\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesFunctionality\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcGetSelButton\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOutputDataPoint\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#outDpSelectButton\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servSelectButtton\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\" ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servTemperature\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Service\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesFunctionality\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcGetTemp\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOutputDataPoint\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#outDpTemperature\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#servVDD\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#Service\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#exposesFunctionality\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#funcGetVoltage\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasOutputDataPoint\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#outDpVoltage\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#uuidInput\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#OperationInput\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasInput\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#deviceUuid\"  } ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasThingProperty\" : [ {    \"@id\" : \"http://yang-netconf-mqtt#uuidYangDesc\"  } ]}, {  \"@id\" : \"http://yang-netconf-mqtt#uuidYangDesc\",  \"@type\" : [ \"http://www.w3.org/2002/07/owl#NamedIndividual\", \"http://yang-netconf-mqtt#YangDescription\" ],  \"http://www.onem2m.org/ontology/Base_Ontology/base_ontology#hasValue\" : [ {    \"@value\" : \"Target UUID for request\"  } ]} ]";

#endif /* ONTOLOGY_H_ */
