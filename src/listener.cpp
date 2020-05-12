//
// Created by sabantu on 5/10/20.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

//    このチュートリアルでは、ROSシステムを介したメッセージの簡単な受信をデモします。
void chatterCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    ros::NodeHandle nodeHandle;

    //    subscribe() コールは、与えられたトピックのメッセージを受信したいことを ROS に伝える方法です。
    //    これはROSマスターノードへの呼び出しを呼び出し、誰が公開していて誰が購読しているかのレジストリを保持します。
    //    メッセージはコールバック関数に渡され、ここでは chatterCallback と呼ばれています。
    //    subscribe() はサブスクライバオブジェクトを返します。
    //    サブスクライバ オブジェクトのすべてのコピーがスコープ外になると、
    //    このコールバックは自動的にこのトピックからサブスクライブを解除します。
    //    subscribe() 関数の 2 番目のパラメータは、メッセージキューのサイズです。
    //    メッセージが処理中よりも速く到着している場合、
    //    これは最も古いメッセージを捨てる前にバッファリングされるメッセージの数です。
    ros::Subscriber subscriber = nodeHandle.subscribe("chatter", 1000, chatterCallback);

    //     ros::spin() はループに入り、コールバックを実行します。 このバージョンでは、
    //     すべてのコールバックはこのスレッド (メインのスレッド) から呼び出されます。
    ros::spin();

    return 0;
}