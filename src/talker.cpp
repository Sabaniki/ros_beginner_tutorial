//
// Created by sabantu on 5/10/20.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
    //    ros::init() 関数は argc と argv を見る必要があるので、コマンドラインで提供された
    //    ROS の引数と名前のリマッピングを実行することができます。プログラムによるリマッピングのためには、
    //    直接リマッピングを行う別のバージョンのinit()を使うことができますが、ほとんどのコマンドラインプログラムでは、
    //    argcとargvを渡すのが最も簡単な方法です。 init() の第三引数はノードの名前です。
    //    ROSシステムの他の部分を使う前に、ros::init()のいずれかのバージョンを呼び出す必要があります。
    ros::init(argc, argv, "talker");

    //    NodeHandleは、ROSシステムとの通信のためのメインのアクセスポイントです。
    //    最初に構築された NodeHandle はこのノードを完全に初期化し、最後に破棄された NodeHandle はノードを閉じます。
    ros::NodeHandle nodeHandle;

    //    advertise() 関数は、指定したトピック名で公開したいことを ROS に伝える方法です。
    //    これは ROS マスターノードへの呼び出しを呼び出します。この advertise() の呼び出しが行われた後、
    //    マスターノードは、このトピック名をサブスクライブしようとしている人に通知し、
    //    順番にこのノードとのピアツーピア接続をネゴシエートします。
    //    返されたパブリッシャオブジェクトのすべてのコピーが破棄されると、そのトピックは自動的に広告されなくなります。
    //    advertise() の 2 番目のパラメータは、メッセージをパブリッシュするために使用するメッセージキューのサイズです。
    //    メッセージを送信するよりも早くメッセージが公開される場合は、ここで指定した数値は、
    //    いくつかのメッセージを捨ててしまう前にバッファリングするメッセージの数を指定します。
    ros::Publisher chatterPub = nodeHandle.advertise<std_msgs::String>("chatter", 1000);

    //     送信したメッセージの数をカウントしたもの。これは、各メッセージに固有の文字列を作成するために使用されます。
    ros::Rate loopRate(10);

    int count = 0;
    while (ros::ok()) {
        //    これはメッセージオブジェクトです。データを詰め込んで公開します。
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        //    publish() 関数はメッセージを送信する方法です。パラメータはメッセージオブジェクトです。
        //    このオブジェクトの型は、上記のコンストラクタで行ったように、
        //    advertise<>()呼び出しのテンプレート・パラメータとして与えられた型と一致していなければなりません。
        chatterPub.publish(msg);

        ros::spinOnce();

        loopRate.sleep();
        ++count;
    }
    return 0;
}