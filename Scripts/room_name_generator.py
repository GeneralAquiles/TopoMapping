#!/usr/bin/env python
# license removed for brevity
import rospy

from std_msgs.msg import *
from topomapping.msg import probability_array
from topomapping.msg import probability_element
#from Vision_AI_parser import 
from subprocess import call

import io
import os
import re
import json
from decimal import Decimal


cantina = {
  "Kitchen & Dining Room Table" : "Exclussive",
  "Kitchen & Dining Room" : "Exclussive",
  "Kitchen" : "Exclussive",
  "Kitchen & dining room table" : "Exclussive",
  "cafeteria" : "Exclussive",
  "Restaurant" : "Exclussive",
  "Table" : "Feature",
  "Desk" : "Feature",
  "Furniture" : "Feature",
  "Chair" : "Feature",
  "Design" : "Feature"
}

pasillo = {
  "Arch" : "Exclussive",
  "Column" : "Exclussive",
  "Staris" : "Exclussive",
  "Beige" : "Feature",
  "Wall" : "Feature"
}

despacho = {
  "Room" : "Exclussive",
  "Door Handle" : "Exclussive",
  "Cabinetry" : "Feature",
  "Interior" : "Feature",
  "Hardwood" : "Feature",
  "Floor" : "Feature",
  "Lock" : "Feature",
  "Chair" : "Feature"
}

clase = {
  "Class" : "Exclussive",
  "Classroom" : "Exclussive",
  "Education" : "Exclussive",
  "Learning" : "Exclussive",
  "Hall" : "Exclussive",
  "Colourfullness" : "Feature",
  "Table" : "Feature",
  "Interior Design" : "Feature",
  "Plywood" : "Feature",
  "Desk" : "Feature",
  "Computer monitor" : "Feature",
  "Office suplies" : "Feature",
  "Gadget" : "Feature",
  "Office equipment" : "Feature",
  "Peripheral" : "Feature",
  "Communication device" : "Feature" 
}

oficina = {
  "Office" : "Exclussive",
  "Office equipment" : "Feature",
  "Table" : "Feature",
  "Desk" : "Feature",
  "Computer desk" : "Feature",
  "Plywood" : "Feature",
  "Output Device" : "Feature",
  "Peripheral" : "Feature",
  "Design" : "Feature",
  "Drawer" : "Feature",
  "Lamp" : "Feature",
  "Display Device" : "Feature",
  "Computer" : "Feature",
  "Computer Monitor" : "Feature"
}

rooms = [{"name" : "cantina", "room_labels" : cantina}, {"name" : "pasillo", "room_labels" : pasillo}, {"name" : "despacho", "room_labels" : despacho}, {"name" : "clase", "room_labels" : clase}, {"name" : "oficina", "room_labels" : oficina}]


def Convert(a):
  x = iter(a)

  res_dct = dict(zip(x, x))
  return res_dct

def Sort_data(labels):

  dict_element = dict()
  sorted_labels=[]
  for label in labels:

    label = re.sub(', ', r"", str(label))
    label = str(label).strip('[')
    label = re.sub(']', r"", str(label))
    label = re.sub(': ', r":", str(label))
    label = re.sub('\n', r"", str(label))
    
    splited_element=[]
    if label: 
      splited_element = label.split(":")
      dict_element[ str(splited_element[0]).replace('"', "") ] = str(splited_element[1]).replace('"', "")
    

    if "topicality" in label:
      sorted_labels.append(dict_element)
      dict_element = dict()

  
  return sorted_labels

def Calculate_probabilities(image_data):
  score = {"cantina" : 0, "pasillo" : 0, "despacho" : 0, "clase" : 0, "oficina" : 0}
  room_probablities = {"cantina" : 0, "pasillo" : 0, "despacho" : 0, "clase" : 0, "oficina" : 0}
  matched_labels = 0
  locked = False 
  matched = False

  sorted_labels = Sort_data(image_data)

  #print(sorted_labels)
  print("Labels detected:")  
  for label in sorted_labels:  print "\t" + label['description'] 
  print "\n"


  for label in sorted_labels:
    matched = False

    for room in rooms:
      if label['description'] in room['room_labels'] and not locked:
        matched = True
        print "Found"
        if room['room_labels'][label['description']] == "Feature":
          print("Found (", label['description'], ") feature for room: ", room['name'])
          score[room['name']] = score[room['name']] + 1 * Decimal(label['score'])

        elif room['room_labels'][label['description']] == "Exclussive":
          print("Found (", label['description'], ") exclusive feature for room: ", room['name'])
          score[room['name']] = score[room['name']] + 1 * Decimal(label['score'])
          locked = True

    if matched: matched_labels = matched_labels + 1

  if matched_labels > 0: 
    for room in rooms: room_probablities[room['name']] = score[room['name']] / matched_labels

  return room_probablities

  


def Analice_image(make_call):
  if make_call:
    
    call("python3 /home/pablo/catkin_ws/src/topomapping/Scripts/Vision_AI_parser.py", shell=True)

    with open('/home/pablo/catkin_ws/src/topomapping/tmp/Vision_AI_output.txt', 'r') as reader:
      labels = reader.readlines()

    room_probablities = Calculate_probabilities(labels)


    print ("\n --------------- \n")

    print ("\nProbabilities: ")

    result = probability_array()
    for room in rooms: 
      print " "+room['name']+":\t"+str(room_probablities[room['name']])
      this_probability = probability_element()
      this_probability.name = room['name']
      this_probability.prob = room_probablities[room['name']]
      result.items.append(this_probability)

    pub.publish(result)
    rate.sleep()

    


if __name__ == '__main__':
    try:
      rospy.init_node('Vision_AI', anonymous=True)
      print ("[INFO]: Vision AI node stablished")
      rate = rospy.Rate(10) # 10hz
      pub = rospy.Publisher('topomapping/room_type', probability_array, queue_size=10)

      if not rospy.is_shutdown():

        rospy.Subscriber('topomapping/upload_photo', Bool, Analice_image)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
