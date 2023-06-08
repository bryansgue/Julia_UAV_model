#!/usr/bin/env julia

#pkg> status ## Es para check los paquetes instalados 
using RobotOS
using LinearAlgebra
using Plots
using Quaternions

@rosimport geometry_msgs.msg: Point, Pose2D, Twist, Pose
@rosimport nav_msgs.msg: Odometry
@rosimport std_msgs.msg: Int32MultiArray
rostypegen()
using .geometry_msgs.msg
using .nav_msgs.msg
using .std_msgs.msg

maxloops = 1000
rosloops = 0

global pos = ones(4)
global vel = zeros(4)


function odo_callback(msg::Odometry)
    quat = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
    q = Quaternions.Quaternion(quat)
    euler = eulerangles(ZYX, q)
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, euler.z]
    vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z]
end

function sensor1_callback(msg::Point) 
    sensor1 =[msg.x, msg.y, msg.z]
end

function sendvalues(pub_obj,vc)  
    msg = Twist()
    msg.linear.x = vc[1]
    msg.linear.y = vc[2]
    msg.linear.z = vc[3]
    msg.angular.z = vc[4]
    publish(pub_obj, msg) 
end

function set_config(pub_config,run,time,hz)  
    msg = Int32MultiArray()
    
    msg.data = [run, time , hz ]

    publish(pub_config, msg) 
end

function main()
    # Espera 1 seg para comenzar la fiesta!!
    rossleep(2)
    
    timerun=30 #Segundos 
    hz = 30 # Frecuencia de actualizacion
    ts = 1/hz
    samples = timerun*hz # datos de muestreo totales
    loop_rate = Rate(hz) # Tiempo de muestre espera
    
    # Inicializacion de matrices
    h = zeros(4, samples)
    vc = zeros(4, samples)
    v = zeros(4, samples)
    hd = zeros(4, samples)
    hdp = zeros(4, samples)
    he = zeros(4, samples)
    ve = zeros(4, samples)

    vcpp = zeros(4, samples)
    psidp = zeros(1, samples)

    J = zeros(4,4)
    global K1 = 0.8*Matrix{Float64}(I, 4, 4)
    global K2 = 0.8*Matrix{Float64}(I, 4, 4)
    global K3 = 0.8*Matrix{Float64}(I, 4, 4)
    global K4 = 0.8*Matrix{Float64}(I, 4, 4)

    t = 0:ts:samples


    global hxd = -3.5*ones(1, samples)
    global hyd = -10.5*ones(1, samples)
    global hzd = 10*ones(1, samples)  
    global psid = (150*(π/180))*ones(1, samples)

    for k in 1:samples
        if k>1
            global psidp[k] = (psid[k]-psid[k-1])/ts
        else
            global psidp[k] = psid[k]/ts
        end
    end

    global psidp[1] = 0
    # Activa controlador para el robot
    set_config(pub_config,1,timerun,hz)  

    global a=0
    global b=0
    
    println("OK, controller is runnig!!!")
    for k in 1:samples 
        
        tic = time()

        hd[:,k] = [hxd[k],hyd[k],hzd[k],psid[k]]
        
        
        h[:,k] = pos
        v[:,k] = vel

        # Ley de Control
        psi = h[4,k]
        J11 = cos(psi);
        J12 = -sin(psi);
        J13 = 0;
        J14 = 0;

        J21 = sin(psi);
        J22 = cos(psi);
        J23 = 0;
        J24 = 0;

        J31 = 0;
        J32 = 0;
        J33 = 1;
        J34 = 0;

        J41 = 0;
        J42 = 0;
        J43 = 0;
        J44 = 1;

        J = [J11 J12 J13 J14;J21 J22 J23 J24;J31 J32 J33 J34;J41 J42 J43 J44]

        # Ley de control minima norma
        he[:,k] = hd[:,k]-h[:,k]
        
        #vc[:,k] = pinv(J) *(hdp[:,k]+K1*tanh.(K2*he[:,k]))
        vc[:,k] = pinv(J) *(K1*tanh.(K2*he[:,k]))

        if k>1
            vcp = (vc[:,k]-vc[:,k-1])/ts
        else
            vcp = vc[:,k]/ts
        end


        # Compensacion dinamica
        x = [1.4189;0.0213;1.2633;-0.2427;1.9647;0.1000;0.1000;0.1000;1.9293;0.7411;-0.0313;0.1000;0.0289;0.8760;0.1000;0.8701;0.1000;0.1000;0.9200]
        w = v[4,k]
        # INERCIAL MATRIX
        M11=x[1];   M12=0;       M13=0;     M14=x[2]; 
        M21=0;      M22=x[3] ;   M23=0;     M24=x[4]; 
        M31=0;      M32=0;       M33=x[5];  M34=0;
        M41=b*x[6]; M42=a*x[7];  M43=0;     M44=x[8]*(a^2 + b^2)+x[9];

        M=[M11 M12 M13 M14; M21 M22 M23 M24; M31 M32 M33 M34; M41 M42 M43 M44];

        #CENTRIOLIS MATRIX
        C11=x[10];     C12=w*x[11];   C13=0;      C14=a*w*x[12];
        C21=w*x[13];   C22=x[14];     C23=0;      C24=b*w*x[15];
        C31=0;         C32=0;         C33=x[16];  C34=0;
        C41=a*w*x[17]; C42=b*w*x[18]; C43=0;      C44=x[19];

        C=[C11 C12 C13 C14; C21 C22 C23 C24; C31 C32 C33 C34; C41 C42 C43 C44];

        # GRAVITATIONAL MATRIX
        G11=0;  G21=0;  G31=0;  G41=0;

        G=[G11;G21;G31;G41];

        ve[:,k] = vc[:,k]-v[:,k]    
        
        #vcp = [0,0,0,0] 
        control = vcp + K3*tanh.(inv(K3)*K4*ve[:,k]);
        vref = M*control+C*vc[:,k]+G;

        # Envia velocidad de manipulavilidad al robot
        sendvalues(pub,vref)
        
        
        rossleep(loop_rate) 
        toc = time()
        dt = toc-tic
        println(h[:,k])
    end

    v_end = [0, 0.0, 0.0, 0] 
    sendvalues(pub,v_end)
    set_config(pub_config,0,timerun,hz)
    
    x = 1:1:samples
    hxe= he[1,1:samples]
    hye= he[2,1:samples]
    hze= he[3,1:samples]
    hpsie= he[4,1:samples]

    plot!(x,hxe, title = "Error de control", label = ["hxe"], lw = 1)
    plot!(x,hye,label = ["hye"], lw = 1)
    plot!(x,hze,label = ["hze"], lw = 1)
    plot!(x,hpsie,label = ["hpsie"], lw = 1)
    savefig("error_posicion.pdf")

end

if ! isinteractive()
    init_node("Controller")  
    pub = Publisher{Twist}("/m100/velocityControl", queue_size=10)
    pub_config = Publisher{Int32MultiArray}("/UAV/Config", queue_size=10)
    
    sub = Subscriber{Odometry}("/dji_sdk/odometry", odo_callback,  queue_size=10)
    #sub2 = Subscriber{Point}("sensor1", sensor1_callback,  queue_size=10)
    
    main()
end

