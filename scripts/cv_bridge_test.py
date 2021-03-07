#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from zed_interfaces.msg import Object
from  zed_interfaces.msg import ObjectsStamped

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color",Image,self.callback_image)
    self.object_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
    self.object_sub = rospy.Subscriber("/zed2/zed_node/obj_det/objects",ObjectsStamped,self.callback_objects)


    self.cmd_vel_pub=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    self.i=0
    self.twist=Twist()

    self.unit_vel=0.2


  def callback_image(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.image=cv_image
    except CvBridgeError as e:
      print(e)
  def callback_depth(self,depth_data):
    try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
    except :
            print('Error in rendeing depth image')
    # depth_imagedepth_data=depth_image

    u = depth_image.shape[0]/2
    v = depth_image.shape[1]/2
    a=depth_image[v]

    self.obstale_pred=[]
    for i in a:
      if i<=1.0:
        self.obstale_pred.append(i)
      
      #print('OBSTACLE PIXELS',len(self.obstale_pred))



  def callback_objects(self,data):
    obj_locs=[]
    obj_img_locs=[]
    obj_vel=[]
    obj_ids=[]
    x1=[]
    x2=[]
    y1=[]
    y2=[]
    images=[]
    if len(data.objects)>0:
      for object in data.objects:
        obj_locs.append(object.position)
        obj_img_locs.append(object.bounding_box_2d)
        obj_vel.append(object.velocity)
        obj_ids.append(object.label_id)
        if object.position[0]<2.0:
          print("PEOPLE ARE NEAR")
          twist=Twist()

          self.cmd_vel_pub.publish(twist)
          return 0

        a=object.bounding_box_2d
        x1=a.corners[0].kp[0]
        y1=a.corners[0].kp[1]
        x2=a.corners[2].kp[0]
        y2=a.corners[2].kp[1]
        print('--')
        img=self.image[y1:y2,x1:x2]
        images.append(img)


      self.id=obj_ids[0]
      localtion=obj_locs[0]


      target_x=localtion[0]
      print('x---',target_x)
      
      target_y=localtion[1]
      target_z=localtion[2]
      

 
      if target_x<2.0 or len(self.obstale_pred)>50:
        print("-------------------------------------------------------------")
        twist=Twist()
        print("OBSTACLE")
        self.cmd_vel_pub.publish(twist)
      else:
        if target_x>2 and target_x<6 and  not len(self.obstale_pred)>50:

          if obj_vel[0][0] >0.1:
            print('FORWARD')
            self.twist.linear.x,self.twist.angular.z=self.unit_vel,0  #obj_vel[0][0],obj_vel[0][
            self.cmd_vel_pub.publish(self.twist)
          elif obj_vel[0][0] <-0.1:
            print("BACKWARD")
            self.twist.linear.x,self.twist.angular.z=-self.unit_vel,0  #obj_vel[0][0],obj_vel[0][1]
            self.cmd_vel_pub.publish(self.twist)
          else:
            print("IDLE")
            twist=Twist()
            self.cmd_vel_pub.publish(twist)

          
        elif target_x>6 and target_x<12 and not len(self.obstale_pred)>50:
          print("REACHING")
          self.twist.linear.x,self.twist.angular.z=self.unit_vel,0  #obj_locs[0][0]/10,obj_locs[0][1]/10
          self.cmd_vel_pub.publish(self.twist)

    else:
            print("**** no person found ****")
            twist=Twist()
            self.cmd_vel_pub.publish(twist)

        

        














      




        


    #     # print(object.label)
    #     # print(object.position)
    #     # print(self.image.shape)
    # print(obj_img_locs)
    # a=obj_img_locs[0]
    # print(a.corners)
    # print(a.corners[0].kp[0])

    # x1=a.corners[0].kp[0]
    # y1=a.corners[0].kp[1]
    # x2=a.corners[2].kp[0]
    # y2=a.corners[2].kp[1]
     
    # img=self.image[y1:y2,x1:x2]
    # print('img shape',img.shape)

    # cv2.imwrite('ram.jpg',img)

  

    





def main(args):
  ic = image_converter()
  rospy.init_node('perso_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
