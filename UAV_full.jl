#!/usr/bin/env julia

#pkg> status ## Es para check los paquetes instalados 
using RobotOS
using LinearAlgebra
using Plots


@rosimport geometry_msgs.msg: Point, Pose2D, Twist, Pose
@rosimport nav_msgs.msg: Odometry
@rosimport std_msgs.msg: Int32MultiArray

rostypegen()
using .geometry_msgs.msg
using .nav_msgs.msg
using .std_msgs.msg

function M_function(chi,q) 
    phi = q[4];
    theta = q[5];
    psi = q[6];
    m1 = chi[1];
    Ixx = chi[2];
    Iyy = chi[3];
    Izz = chi[4];

    M = [m1 0 0 0 0 0;
    0 m1 0 0 0 0;
    0 0 m1 0 0 0;
    0 0 0 Ixx 0 -Ixx*sin(theta);
    0 0 0 0 Izz+Iyy*cos(phi)^2-Izz*cos(phi)^2 Iyy*cos(phi)*cos(theta)*sin(phi)-Izz*cos(phi)*cos(theta)*sin(phi);
    0 0 0 -Ixx*sin(theta) Iyy*cos(phi)*cos(theta)*sin(phi)-Izz*cos(phi)*cos(theta)*sin(phi) Ixx-Ixx*cos(theta)^2+Iyy*cos(theta)^2-Iyy*cos(phi)^2*cos(theta)^2+Izz*cos(phi)^2*cos(theta)^2];

    return M
end

function C_function(chi,q,q_p) 
    phi = q[4];
    theta = q[5];
    psi = q[6];

    phi_p = q_p[4];
    theta_p = q_p[5];
    psi_p = q_p[6];

    m1 = chi[1];
    Ixx = chi[2];
    Iyy = chi[3];
    Izz = chi[4];
    
    C = [0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 (Iyy*psi_p*cos(theta))/2 - (Ixx*psi_p*cos(theta))/2 - (Izz*psi_p*cos(theta))/2 - Iyy*psi_p*cos(phi)^2*cos(theta) + Izz*psi_p*cos(phi)^2*cos(theta) + Iyy*theta_p*cos(phi)*sin(phi) - Izz*theta_p*cos(phi)*sin(phi) (Iyy*theta_p*cos(theta))/2 - (Ixx*theta_p*cos(theta))/2 - (Izz*theta_p*cos(theta))/2 - Iyy*theta_p*cos(phi)^2*cos(theta) + Izz*theta_p*cos(phi)^2*cos(theta) - Iyy*psi_p*cos(phi)*cos(theta)^2*sin(phi) + Izz*psi_p*cos(phi)*cos(theta)^2*sin(phi);
    0 0 0 (Ixx*psi_p*cos(theta))/2 - (Iyy*psi_p*cos(theta))/2 + (Izz*psi_p*cos(theta))/2 + Iyy*psi_p*cos(phi)^2*cos(theta) - Izz*psi_p*cos(phi)^2*cos(theta) - Iyy*theta_p*cos(phi)*sin(phi) + Izz*theta_p*cos(phi)*sin(phi) Izz*phi_p*cos(phi)*sin(phi) - Iyy*phi_p*cos(phi)*sin(phi) (Ixx*phi_p*cos(theta))/2 - (Iyy*phi_p*cos(theta))/2 + (Izz*phi_p*cos(theta))/2 + Iyy*phi_p*cos(phi)^2*cos(theta) - Izz*phi_p*cos(phi)^2*cos(theta) - Ixx*psi_p*cos(theta)*sin(theta) + Iyy*psi_p*cos(theta)*sin(theta) - Iyy*psi_p*cos(phi)^2*cos(theta)*sin(theta) + Izz*psi_p*cos(phi)^2*cos(theta)*sin(theta);
    0 0 0 (Izz*theta_p*cos(theta))/2 - (Iyy*theta_p*cos(theta))/2 - (Ixx*theta_p*cos(theta))/2 + Iyy*theta_p*cos(phi)^2*cos(theta) - Izz*theta_p*cos(phi)^2*cos(theta) + Iyy*psi_p*cos(phi)*cos(theta)^2*sin(phi) - Izz*psi_p*cos(phi)*cos(theta)^2*sin(phi) (Izz*phi_p*cos(theta))/2 - (Iyy*phi_p*cos(theta))/2 - (Ixx*phi_p*cos(theta))/2 + Iyy*phi_p*cos(phi)^2*cos(theta) - Izz*phi_p*cos(phi)^2*cos(theta) + Ixx*psi_p*cos(theta)*sin(theta) - Iyy*psi_p*cos(theta)*sin(theta) + Iyy*psi_p*cos(phi)^2*cos(theta)*sin(theta) - Izz*psi_p*cos(phi)^2*cos(theta)*sin(theta) - Iyy*theta_p*cos(phi)*sin(phi)*sin(theta) + Izz*theta_p*cos(phi)*sin(phi)*sin(theta) Ixx*theta_p*cos(theta)*sin(theta) - Iyy*theta_p*cos(theta)*sin(theta) + Iyy*phi_p*cos(phi)*cos(theta)^2*sin(phi) - Izz*phi_p*cos(phi)*cos(theta)^2*sin(phi) + Iyy*theta_p*cos(phi)^2*cos(theta)*sin(theta) - Izz*theta_p*cos(phi)^2*cos(theta)*sin(theta)];

    return C
end

function G_function(chi)    
    g = 9.81;
    m1 = chi[1];   
    G = [0; 0; g*m1; 0; 0; 0];
    return G
end

function S_function(chi)    
    S = zeros(6, 6)
    S[3,3] = chi[5];
    S[4,4] = chi[6];
    S[5,5] = chi[7];
    S[6,6] = chi[8];
    return S
end

function Q_function(chi)    
    Q = zeros(6, 6)
    Q[4,4] = chi[9];
    Q[5,5] = chi[10];
    Q[6,6] = chi[11];
    return Q
end

function E_function(chi)    
    E = zeros(6, 6)
    E[3,3] = chi[12];
    E[4,4] = chi[13];
    E[5,5] = chi[14];
    E[6,6] = chi[15];
    return E
end

function T_function(chi)    
    T = zeros(6, 6)
    T[3,3] = chi[16];
    return T
end

function R_zyx(euler)    
    
    phi = euler[1];
    theta =euler[2];
    psi = euler[3];

    RotX = [1 0 0;
            0 cos(phi) -sin(phi);
            0 sin(phi) cos(phi)];
        
    RotY = [cos(theta) 0 sin(theta);
            0 1 0;
            -sin(theta) 0 cos(theta)];
        
    RotZ = [cos(psi) -sin(psi) 0;
            sin(psi) cos(psi) 0;
            0 0 1];
    
    R = RotZ*RotY*RotX;
    return R
end

function RK4_UAV(s,u,chi,Ts)    
    
    k1 = dinamic_system(s, u, chi);
    k2 = dinamic_system(s + (Ts/2)*k1, u, chi);
    k3 = dinamic_system(s + (Ts/2)*k2, u, chi);
    k4 = dinamic_system(s + Ts*k3, u, chi);
    s_sig = s + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
    return s_sig
end


function dinamic_system(s,inputs,chi)    
    # GRAVITATIONAL MATRIchi
    g = 9.81;
    m = chi[1];

    q = s[1:6,1];
    q_p = s[7:12,1];

    M= M_function(chi,q) 
    C= C_function(chi,q,q_p) 
    G= G_function(chi) 

    S = S_function(chi);
    Q = Q_function(chi);
    E = E_function(chi);
    T = T_function(chi);
    B = [0;0;m*g;0;0;0];
    
    euler = q[4:6,1]
    R = R_zyx(euler)
    
    R_T = [R*T[1:3,1:3] T[1:3,4:6];T[4:6,1:3] T[4:6,4:6]];
    Aux = S*inputs-Q*q-E*q_p+B;
    Aux1 = R*Aux[1:3,1];
    Aux2 = Aux[4:6,1];
    Input_model = [Aux1;Aux2];
    q_pp = inv(M+R_T)*(Input_model-C*q_p-G);
    
    s_p = [q_p; q_pp];
    return s_p
end

function graficar_vectores1(s)
    vector1 = s[1,:]
    vector2 = s[2,:]
    vector3 = s[3,:]
    
    p1 = plot(1:length(s[1,:]), vector1, label="hx")
    plot!(1:length(s[2,:]), vector2, label="hy")
    plot!(1:length(s[3,:]), vector3, label="hz")
    title!("Grafica de 3 vectores")
    xlabel!("Eje x")
    ylabel!("Eje y")

    plot(p1, layout=(1,1), size=(800,400))

    savefig("POS_xyz.pdf")
end

function graficar_vectores2(s)
    vector1 = s[4,:]
    vector2 = s[5,:]
    vector3 = s[6,:]
    
    p1 = plot(1:length(s[1,:]), vector1, label="phi")
    plot!(1:length(s[2,:]), vector2, label="theta")
    plot!(1:length(s[3,:]), vector3, label="psi")
    title!("EULER")
    xlabel!("Eje x")
    ylabel!("Eje y")

    plot(p1, layout=(1,1), size=(800,400))
    savefig("Pos_euler.pdf")
end

function graficar_vectores3(s)
    vector1 = s[7,:]
    vector2 = s[8,:]
    vector3 = s[9,:]
    
    p1 = plot(1:length(s[1,:]), vector1, label="hx_p")
    plot!(1:length(s[2,:]), vector2, label="hy_p")
    plot!(1:length(s[3,:]), vector3, label="hz_p")
    title!("Velocidades Lineales")
    xlabel!("Eje x")
    ylabel!("Eje y")

    plot(p1, layout=(1,1), size=(800,400))
    savefig("Vel_h_p.pdf")
end

function graficar_vectores4(s)
    vector1 = s[10,:]
    vector2 = s[11,:]
    vector3 = s[12,:]
    
    p1 = plot(1:length(s[1,:]), vector1, label="phi_p")
    plot!(1:length(s[2,:]), vector2, label="theta_p")
    plot!(1:length(s[3,:]), vector3, label="psi_p")
    title!("EULER")
    xlabel!("Eje x")
    ylabel!("Eje y")

    plot(p1, layout=(1,1), size=(800,400))
    savefig("Euler_p.pdf")
end

function Load_chi()    
    chi = 1.0e-05 * [0.0011; 0.0017;0.0012;0.0058; 0.0062;0.1497;0.0842;0.1340;0.1338;0.0732;0.1307;0.0061;0.0308;0.0180;0.0425;0.0061]
    
    return chi
end

function vc_callback(pose::Twist) 
    global v_ref =[0, 0 , pose.linear.x, pose.linear.y, pose.linear.z, pose.angular.z]
end

function set_odo(pub_odo,s,frame_id)  
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = s[1]
    msg.pose.pose.position.y = s[2]
    msg.pose.pose.position.z = s[3]
    msg.pose.pose.orientation.x = s[4]
    msg.pose.pose.orientation.y = s[5]
    msg.pose.pose.orientation.z = s[6]
    msg.twist.twist.linear.x = s[7]
    msg.twist.twist.linear.y = s[8]
    msg.twist.twist.linear.z = s[9]
    msg.twist.twist.angular.x = s[10]
    msg.twist.twist.angular.y = s[11]
    msg.twist.twist.angular.z = s[12]

    publish(pub_odo, msg) 
end

function main()

    # Espera arranque
    println("Run the controller, please!")
    
    t = 60;
    Hz = 30 # Frecuencia de actualizacion
    samples = t*Hz; # datos de muestreo totales
    loop_rate = Rate(Hz); # Tiempo de muestre espera
 
    # Inicializacion de matrices
    q = zeros(6, samples+1)
    q_p = zeros(6, samples+1)
    vref = zeros(6, samples+1)
    s = zeros(12, samples+1)
    s_p = zeros(12, samples+1)
        
    # Estados iniciales del robot
    hx = 0;
    hy = 0;
    hz = 1;
    roll = 0;
    pitch = 0;
    yaw = 0;
    q[:,1] = [hx hy hz roll pitch yaw]

    hx_p = 0;
    hy_p = 0;
    hz_p = 0;
    roll_p = 0;
    pitch_p = 0;
    yaw_p = 0;

    q_p[:,1] = [hx_p hy_p hz_p roll_p pitch_p yaw_p]

    s[:,1] = [q[:,1]; q_p[:,1]]

    println("OK, controller is runnig!!!")

    #Begins the controller
    for k in 1:(t*Hz) 
        
        tic = time()
        ts = (1/Hz);
        a=0;
        b=0;
        
        chi = Load_chi();

        try
            vref[:,k] = v_ref
        catch
            aux_zp =    0.02; 
            zref_p =     00.00;
            phid =      0.001;
            thetad =    0.00;
            psid =      0.0;
            
            vref[:,k] = [0.0, 0.0, zref_p, phid, thetad, psid]  
        end 

        #s_p = dinamic_system(chi,s[:,k],vref[:,k]);

        s[:,k+1] = RK4_UAV(s[:,k],vref[:,k],chi,ts) ;
      
        # Enviar valores de posicion y velocidad (Odometria)
        set_odo(pub_pos, s[:,k+1],"input")

        println(k)
        rossleep(loop_rate) 
        toc = time()
        dt = toc-tic
    end    
    println("MODEL HAS FINISHED!!")

# Graficamos cada vector en una subfigura diferente y guardamos en PDF


    graficar_vectores1(s)
    graficar_vectores2(s)
    graficar_vectores3(s)
    graficar_vectores4(s)
    

end

if ! isinteractive()
    init_node("Robot")  
    sub_vc  = Subscriber{Twist}("/UAV/Controller", vc_callback,  queue_size=10)
    #sub_run = Subscriber{Int32MultiArray}("/UAV/Config", config_callback,  queue_size=10)
    
    # Lanza los publisher del sistema
    pub_config = Publisher{Int32MultiArray}("/UAV/Config", queue_size=10)
    pub_pos = Publisher{Odometry}("/UAV/Odometry", queue_size=10)
    pub = Publisher{Twist}("/UAV/Controller", queue_size=10)
    main()
end