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
count__ = 0 
def crop_sign(image):
    cv_image = image

    (rows, cols, channels) = cv_image.shape
    #imCrop2 = cv_image[0:240,400:640]
    imCrop2 = cv_image
    hsv = cv2.cvtColor(imCrop2, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 70, 50])
    upper_blue = np.array([120, 255, 255])
    lower_red = np.array([0, 50, 0])
    upper_red = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 0])
    upper_red2 = np.array([180, 255, 255])
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_r1 = cv2.inRange(hsv, lower_red, upper_red)
    mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask_r = mask_r1 + mask_r2
    #mask = mask_r + mask_b
    mask = mask_b
    image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if (len(contours) > 0):
        max_area = 0
        ci = 0
        for i in range(len(contours)):
            cnt = contours[i]
	    #print(contours[i])
            area = cv2.contourArea(cnt)
            if (area > max_area):
                max_area = area
                ci = i

        cnt = contours[ci]
	#print(cnt)

        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        x, y, w, h = cv2.boundingRect(approx)
	
        if w > 50 and h > 50 and w / h >= 0.6 and w / h <= 1.8:
            for i in range(0, 5) :
		mask = cv2.dilate(mask, np.ones((int(w/10), int(h/10)), np.uint8), iterations=1)
	    for i in range(0, 5) :
		mask = cv2.erode(mask, np.ones((int(w/10), int(h/10)), np.uint8), iterations=1)
	    imCrop2 = cv2.bitwise_and(imCrop2, imCrop2, mask=mask)
            a = y
            b = y + h
            c = x
            d = x + w
	    
            imCrop = imCrop2[a:b, c:d]

            return imCrop

def talker(classification, img):
	global prev_index, buff, count__
	
	if not rospy.is_shutdown():
		rospy.loginfo(classification)
		cur_index = labels.index(classification)
		#filename = "/home/nvidia/ISCC/" + str(cur_index) + "/" + str(count__) + ".jpg"
		count__ = count__ + 1
		#cv2.imwrite(filename, img)
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
				img = cv2.flip(img,0)
				img = cv2.flip(img,1)
				img_crop = img[0:240, 400:640]
                		img_detected = crop_sign(img_crop)
				if img_detected is None :
			                continue	

		                if ret_val:
                		        start_time = time.time()

		                        predictions = sess.run(softmax_tensor, {'DecodeJpeg:0': img_detected})

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
					if(max_score > 0.5) :
						talker(classification, img_detected)

                        	if cv2.waitKey(1) & 0xFF == ord('q'):
                                	break
				rate.sleep()

	except rospy.ROSInterruptException: pass
