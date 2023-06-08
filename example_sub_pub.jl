#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D, Twist
rostypegen()
using .geometry_msgs.msg

function callback(msg::Twist, pub_obj::Publisher{Point})
    global hdp
    hpd = [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z]
    println(hpd)
    
end

function sendvalues()
    pub = Publisher{Point}("publicaDatos", queue_size=10)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        npt = Point(rand(), rand(), 0.0)
        publish(pub, npt)
        rossleep(loop_rate)
    end
    
end

function listener()
    sub = Subscriber{Twist}("Datos", callback, (pub,), queue_size=10)
    
end


function main()
    
    
    sendvalues()
    
end

if ! isinteractive()
    init_node("ListenerPublisher")
    listener()
    main()
end  
    