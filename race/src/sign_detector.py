#!/usr/bin/env python

import rospy
import tensorflow as tf
import sys
import cv2
import time
import win_unicode_console
win_unicode_console.enable()
from PIL import Image
import io
from race.msg import sign_classes
from std_msgs.msg import Bool

sc_onoff = False
prev_onoff = False

labels = ['no sign', 'crosswalk', 'static obstacle', 'dynamic obstacle', 'branch road', 'curve', 'uturn', 'parking']

buff = 0
prev_index = -1
cam = cv2.VideoCapture()

def talker(classification):
	global prev_index, buff
	
	if not rospy.is_shutdown():
		rospy.loginfo(classification)
		cur_index = labels.index(classification)
		if cur_index != prev_index :
			buff = 0
			prev_index = cur_index
		else :
			buff = buff + 1
		if buff == 5 :
			pub.publish(cur_index)
			buff = 0		
def sc_onoffCallback(data) :
	global prev_onoff, sc_onoff, cam
	sc_onoff = data.data
	if(prev_onoff != sc_onoff) :
		prev_onoff = sc_onoff
		if(sc_onoff) :
			print("SC ON")
			cam.open(3)
		else :
			print("SC OFF")
			cam.release()

if __name__ == '__main__' :
	pub = rospy.Publisher('/sign_classes', sign_classes, queue_size = 10)
	rospy.init_node('sign_detector')
	rospy.Subscriber("sc_onoff", Bool, sc_onoffCallback)
	rate = rospy.Rate(10) # 10hz
	try:
		with tf.gfile.FastGFile("work/signs/retrained_graph.pb", 'rb') as f:
	        	graph_def = tf.GraphDef()
        		graph_def.ParseFromString(f.read())
        		_ = tf.import_graph_def(graph_def, name = '')
			
		with tf.Session() as sess:
			label_lines = [line.rstrip() for line in tf.gfile.GFile("work/signs/retrained_labels.txt")]
		        softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
       			while True :
				if(not sc_onoff or not cam.isOpened()) :
					continue
                		ret_val, img = cam.read()
				img_crop = img[0:320, 384:640]
                		cv2.imshow('img', img)
				cv2.imshow('img_crop', img_crop)
			

		                if ret_val:
                		        start_time = time.time()

		                        predictions = sess.run(softmax_tensor, {'DecodeJpeg:0': img})

                		        top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
		                        print("--- %s seconds ---" %(time.time() - start_time))

                		        max_score = 0

                		        for node_id in top_k:
                                		human_string = label_lines[node_id]
		                                score = predictions[0][node_id]
		                                print('%s (score = %.5f)' % (human_string, score))
                                
                		                if score > max_score:
                                		        max_score = score
		                                        classification = human_string


                		        print(classification)
		                        print()
					talker(classification)

                        	if cv2.waitKey(1) & 0xFF == ord('q'):
                                	break
				rate.sleep()

	except rospy.ROSInterruptException: pass
