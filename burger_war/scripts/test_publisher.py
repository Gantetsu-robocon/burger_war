#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy
from geometry_msgs.msg import PoseStamped 

def para_in():
    # 初期化宣言 : このソフトウェアは"para_in"という名前
    rospy.init_node('para_in', anonymous=False)

    # nodeの宣言 : publisherのインスタンスを作る
    # input_dataというtopicにAdder型のmessageを送るPublisherをつくった
    pub = rospy.Publisher('relative_pose', PoseStamped, queue_size=100)

    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    para_x = 0.0
    para_y = 0.0
    para_theta = 1.0

    # Adder型のmessageのインスタンスを作る
    msg = PoseStamped()

    # ctl +　Cで終了しない限りwhileループでpublishし続ける

    while not rospy.is_shutdown():

        msg.pose.position.x = para_x
        msg.pose.position.y = para_y
        msg.pose.orientation.w = para_theta

        # publishする関数
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        para_in()

    except rospy.ROSInterruptException: pass