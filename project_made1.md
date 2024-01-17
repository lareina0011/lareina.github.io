![Alt text](image.png)
# project_made1项目报告
## 第二组 组员：王皓 江建波 金文亮 
## 时间：2024/01/17
## task
1. ROS工作空问的创建
2. 基于topic通信
3. 基于service的通信
4. 基于激光雷达的小车避障
5. 基于OpenCV的小车循迹
### task1
1.连接小车上的ros系统,可以采用vscode的ssh远程连接,或者直接在终端连接。
2. ROS创建工作空间：首先在vscode中打开终端，并为自己的工作空间命名,输入命令mkdir -p ....。例如我们组的工作空间为jubot_ws
3. 编译：使用catkin_make命令即可，然后等待编译完成即可.(ros2采用的命令为colon build)
4. 运行：运行前需要先source一下工作空间，source jubot_ws/devel/setup.bash
### task2
![Alt text](1705468159251.png)
1.ros的通信机制：首先是talker与listener的注册，rosmaster会进行信息的匹配，在收到master发挥的talker的地址消息，listener会发送连接请求，talker确定连接请求，然后两者建立网络连接，之后talker便可以向listener发送消息，listener便可以接收到消息。
2.实验部分：首先需要创建一个package，catkin_create_pkg topic std_msgs rospy roscpp,然后重新回到工作空间编译。进入包内我们可以touch talker.py，在程序写好后需要chomd +x touch.py,让其变成一个可执行文件。同理也可以建一个接收者touch listener.py。
3.我们可以通过rostopic来查看topic的信息，例如rostopic echo chatter,可以查看chatter的信息。想查看话题的具体内容我们可以通过rostopic echo -n 10 chatter来查看最近10条消息。
4.我们可以通过rosrun来运行程序，例如rosrun topic talker,rosrun topic listener,这样就可以运行程序了。我们可以在终端看到如下内容
```python
import rospy
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
   pass
```
![Alt text](012a3a6c34b9bebdfd7c67a3a344ffd.png)
### task3 
![Alt text](b2860d865173e92696861fd5238ace8.png)
这里前面的步骤和task2一样的我们就不重复了，我们主要讲一下task3的实验内容。
1.首先我们要建立一个srv文件，然后可以通过vim或者nano编辑器进行编辑，当然vscode也可以直接编辑。这里我们简单的只写一个求俩数之和
2.这里与task2不同的是，我们还需要在package的CMakeLists.txt中添加一些内容，如下所示：
```python
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(      
  FILES  
  add_two_ints.srv      
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

```
3.然后我们还需要在package的src/CMakeLists.txt中添加一些内容，如下所示：
```python
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})

```
4.然后我们还需要在package的src/add_two_ints_server.cpp中添加一些内容(这里如果我们建立的是.py文件我们还需要在cmake里面的python install里面进行下载)，如下所示：
```python
#include "ros/ros.h"
#include "homework_service/AddTwoInts.h"
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}

```
5.然后我们还需要在package的src/add_two_ints_client.cpp中添加一些内容，如下所示：
```python
#include "ros/ros.h"
#include "homework_client/AddTwoInts.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  if (ros::service::call("add_two_ints", srv))
  {
    ROS_INFO("sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}

```
6.接下来，我们就可以在package的根目录下运行命令：
```python
catkin_make
```
7.然后，我们就可以运行这个package了，如下所示：
```python
rosrun beginner_tutorials add_two_ints_server
rosrun beginner_tutorials add_two_ints_client 1 3
```
![Alt text](d0a9b10fadb00d3fc230dcc352817b5.png)
### task4
1.在做巡线这个任务的时候我们通过rostopic info命令发现没有话题发布者，这个部分是需要自己写的，具体怎么写可以参见task2
2.在这个任务正式开始之前我们需要完成以下的准备工作：a.打开rosmaster b.打开底盘控制的驱动 roslauch jubot_driver jubot_driver.launch c.打开摄像头控制 roslaunch jubot_vision jubot_vision.launch 
3.接下来我们就可以开始写代码了，和前面的task2和task3一样，我们先新建一个程序包我们命名为homework_searchline ,然后就可以建立touch homework_searchline.py，touch homework_followline.py，完成代码内容.在我们实践过程中我们发现了首先由于地板的原因会反射光线，影响摄像头图片的获取，以及转变成hsv图像和灰度图像时，由于光线的原因，灰度图像的值会发生变化，所以我们需要对图像进行预处理并且我们通过foxglove发现我们红点的位置太高了因此我们将位置调成了4/5.
``` python
# !/usr/bin/env python
# -*- coding: UTF-8 -*-
#引入相关的包
import rospy,cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
class Follower :
#类的构造方法
    def __init__(self) :
# 创建cv_bridge对象，用于opencv图像与ros图像的相互转换
        self.bridge = cv_bridge .CvBridge()
#订阅摄像头的输出话题，在名为image_callback的回调函数中逐顿处理图像
        self.image_sub = rospy. Subscriber( ' camera/rgb/image_raw' ,Image, self.image_callback)
#定义原始图像、hsv图像、mask图像的发布者
        self.ori_pub = rospy.Publisher( 'ori' , Image, queue_size=1)
        self.hsv_pub = rospy.Publisher( 'hsv', Image, queue_size=1)
        self.mask_pub = rospy.Publisher ('mask' , Image, queue_size=1)#摄像头图像话题的回调函数
    def image_callback (self, msg) :
# 使用cv_bridge将接到的ros图像转换为opencv图像格式
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8' )#将原始ros图像发布，用于rviz订阅
        self.ori_pub.publish(msg)
# 使用opencv将BGR图像转换为HSV图像
        hsv = cv2.cvtColor(image, cV2.COLOR_BGR2HSV)
        try:
# 使用cv_bridge将opencv图像转换为ros图像，发布hsv图像话题
            self.hsv_pub.publish(self.bridge.cv2_to_imgmsg(hsv))
        except cv_bridge.CvBridgeError as e :
            print(e)
#定义黄色的hsv闻值，同学们可上网自行查阅每种颜色所对应的hsv范围
        lower_yellow = numpy.array([ 26, 43,46])
        upper_yellow = numpy.array( [34,255 ,255])#根据黄色值将hsv图像中的黄色区域提取出来#原黄色区域像素值为255，显示白色，其余区域像素值为0，显示黑色mask = cv2 .inRange (hsv, lower_yellow, upper_yellow)# 对image与mask图像执行bitwise_and操作
# image图像上对应于mask图像o值的像素位将被置0，其余像素则保留原值#即提取出了image图像上黄色路径的部分
        masked = cv2.bitwise_and(image, image, mask=mask)
        try:
# 将mask图像发布到mask话题中

            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask))
        except cv_bridge.CvBridgeError as e:
            print(e)

# 初始化follower节点
rospy .init_node( ' follower' )
# 创建FolLower类的示例
follower = Follower()
#不断触发回调函数
rospy .spin()
```
```python
# !/usr/bin/env python
# -*- coding: UTF-8 -*-
#引入相关的包
import rospy,cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs .msg import Twist
class Follower :
# 类的构造方法
    def __init__(self):
# 创建cv_bridge对象，用于opencv图像与ros图像的相互转换
        self.bridge = cv_bridge.CvBridge()
#订阅摄像头的输出话题，在名为image_callback的回调函数中逐顿处理图像
        self,image_sub = rospy .Subscriber(' camera/rgb/image_raw' ,Image, self.image_callback)
#发布小车速度话题cmd_vel
        self.cmd_vel_pub = rospy.Publisher( ' cmd_vel' ,Twist, queue_size=1)
#发布绘制了瞄点的图像
        self.circle_pub = rospy.Publisher(' circle', Image, queue_size=1)
        self.twist = Twist()
#摄像头图像话题的回调函数
    def image_callback(self, msg) :
# 图像颜色过滤操作，参考follower_color_filter.py脚本
#根据黄色的hsv颜色范围提取图像中的黄色路径
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding=' bgr8')
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 26 , 43,46])
        upper_yellow = numpy.array( [34,255 ,255])
        mask = cv2 .inRange (hsv , lower_yellow, upper_yellow)
#获取image的高、宽与通道，并分别赋值给h，w，d变量
        h , w, d = image.shape
#定义瞄点区域，为图像3/4处往下20行以内
        search_top = 4*h/5
        search_bot = search_top + 20
#将mask图像中超出不范围的9部分家B0RJ￥产
        mask[0:search_top, :w] = @
        mask[search_bot:h, :w] = O
# 用cv2.moments算子计算图像的矩
        M = cv2.moments(mask)
# M[“moo”]为@阶矩，表示图像连通域的面积
        if M['moo'] >0:
# M[“m10]与M[“m1为1阶矩
# 1阶距代表图像白色区域上x和y坐标值的累计和
# 除以白色区域的面积，求白色区域像素的平均x与y值，即为白色区域的重心
# 坐标:( cx,cy )。
            cx = int(M['m10']/M['mOo'])
            cy = int(M['m01']/M['moo'])
#在重心处绘制半径为2@像素的红色实心圆形，即为瞄点
            cv2 .circle(image, (cx, cy),20,,,255),-1)#计算连通域重心到整幅图像中心x方向的偏差值
            err = Cx - w/2
#根据偏差值计算角速度，发布速度消息
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel pub .publish( self .twist)
#若连通域面积为0，则发布原地旋转的速度消息
        else :
            self.twist.linear.x = 0
            self.twist.angular.z = 0.3
            self.cmd_vel_pub .publish( self.twist)
        try:
#发布绘制了瞄点的图像
            self.circle_pub .publish(self.bridge. cv2_to_imgmsg(image))
        except cv_bridge.CvBridgeError as e:
            print(e)
rospy .init_node('follower' )
follower = Follower()
rospy .spin()
```

4.接下来我们就可以运行这个程序包了，如下所示：
```python
rosrun homework_searchline homework_searchline.py
rosrun homework_searchline homework_followline.py
```
### task5
1.前面的步骤我们可以参照task2和task3，就是新建一个程序包，然后重新返回工作空间进行编译
2.在这个任务正式开始之前我们需要完成以下的准备工作：a.打开rosmaster b.打开底盘控制的驱动 roslauch jubot_driver jubot_driver.launch c.打开雷达驱动控制 roslaunch rplidar_ros view_rplidar.launch 
3. 接下来我们就可以开始写代码了，和前面的task2和task3一样，我们先新建一个程序包我们命名为homework_lidar ,然后就可以建立touch lidar.py，touch homework_followline.py，完成代码内容.代码如下所示
``` python
#!/usr/bin/env python#-*- coding: UTF-8 -*-import rospy
from geometry_msgs msg import Twist
#定义速度话题发布者
cmd_vel_pub=rospy.Publisher ( ' cmd_vel',Twist,queue_size=1)#初始化红绿灯节点
rospy.init_node('red light_green _light')
# 分别定义红绿灯速度消息
red_light_twist=Twist()
green_light_twist=Twist()
#前进标志，初值为false
deriving_forward=False
#绿灯前进
green_light_twist.linear.x=0.5# 红灯后退
red_light_twist.linear.x=-0.5
#定义运动模式切换时间为3秒后
light_change_time=rospy.Time .now()+rospy .Duration(3#定义循环频率为10hz
rate=rospy.Rate(10)
while not rospy.is_shutdown() :
#前进标志为true时发布绿灯速度消息
    if deriving_forward :
        cmd_vel_pub.publish(green_light_twist)
        #否则发布红灯速度消息
    else:
    cmd_vel_pub.publish(red light_twist)#如果过了运动模式的切换时间
    if light_change_time < rospy.Time.now():
        #运动标志置反
    deriving_forward= not deriving_forward
    #更新运动模式切换时间
    light_change_time= rospy.Time.now()+rospy.Duration(3)
    rate.sleep()
```
``` python
#!/usr/bin/env python
#-*- coding;utf-8一光
import rospy
from geometry_msgs.msg import Twistfrom sensor_msgs.msg import LaserScan
# 处理scan话题数据的回调函数
#将前方障碍物距离存入全局变量  
def scan_callback(msg):
    global range_ahead
    range_ahead=msg.ranges[o]
    print(range_ahead)
    range_ahead=0
# 初始化节点
rospy .init node('wander')
# 订阅Gazebo仿真环境中机器人激光扫描仪发出的scan话题
scan_sub=rospy.Subscriber('scan',LaserScan,scan callback)# 创建名为cmd_vel，类型为Twist的cmd_vel_pub话题# queue_size 缓存消息队列大小
cmd_vel_pub=rospy.Publisher(' cmd_vel',Twist, queue_size=1)speed = Twist()
ate=rospy.Rate(10)
#根据前方障碍物距离更改机器人的动作while not rospy.is_shutdown() :
#当前方障碍距离大于1米时，前进
if range_ahead>1:
    speed.linear.x = 0.4
    speed.angular .z = 0
    cmd vel pub.publish(speed)
#当前方障碍距离小于0.4米时，后退
elif range_ahead<0.4:
    speed.linear.x = -0.4
    speed ,angular.z = 0
    cmd vel pub.publish(speed)#当前方距离在0.4米和1米之间时，原地旋转
else:
    speed.linear.x=0
    speed.angular .z=-0.5
    cmd_vel_pub.publish(speed)
rate.sleep()
```
4.接下来就可以运行我们的程序包了
``` python
rosrun homework_lidar homework_lidar_node.py
```
![Alt text](1705489150947.png)

