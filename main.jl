 #connexion avec VREP
PATHCSIM="/home/lucas/Documents/Polytech/S7/Modelisation3D/MiniProjet/";
include("/home/lucas/Documents/Polytech/S7/Modelisation3D/MiniProjet/lib-robotique.jl"); 
CreateRobotKukaLwr()
include("/home/lucas/Documents/Polytech/S7/Modelisation3D/MiniProjet/lib-CSim.jl")

global clientID=startsimulation(simx_opmode_oneshot) # On lance une instance de connexion avec VREP

if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end

T7=zeros(4,4);
init_pos()

global rob=CreateRobotKukaLwr();
global θinit=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
global θmin=(pi/180)*[-170, -120, -170, -120, -170, -120, -170];
global θmax=(pi/180)*[170, 120, 170, 120, 170, 120, 170];
global ωmax=(pi/180)*[112.5, 112.5, 112.5, 112.5, 180, 112.5, 112.5];
global pA=[-0.3668, -0.0379, 0.8634];
global pB=[-0.3668, -0.0379, 0.5];
global K=1;
dP = [0.0, 0.0, -0.01];
dt = 0.05;

function inverseKinematics(θi, Pf, rob, dP, dt)
    q = θi;
    q_collect = Vector{Vector{Float64}}()
    push!(q_collect, copy(q))
    Pe = MGD(q, rob)[1][1:3,4];
    i=0;
    while norm(Pf-Pe) > 1e-3 && i < 200
        t0 = time()
        Pe = MGD(q, rob)[1][1:3,4];
        ϵP = Pf .- Pe;
        J = Jacobian(q, rob, Pe)[1:3,:];
        dq = pinv(J)*(K*ϵP .+ dP);
        q = q + dq*dt;
        push!(q_collect, copy(q))
        #t1 = time()     
        setjointposition(clientID,q,7,0,objectname_kuka) 
        i=i+1;
        t2 = time()
        #println("Temps de calcul : $(t1 - t0) secondes")
        #println("Temps VREP : $(t2 - t1) secondes")
        #println("Reste à attendre : $(dt-(t2-t0)) secondes")
        sleep(max(0.0,round(dt-(t2-t0), digits=4)))
        #println("Itération $i : Position actuelle = $Pe, Erreur = $(norm(Pf-Pe))")
    end
    return q_collect
end

# Robot en position initiale
setjointposition(clientID,θinit,7,0,objectname_kuka)
sleep(3)

q_collect = inverseKinematics(θinit, pB, rob, dP, dt)

stopsimulation(clientID,simx_opmode_oneshot) # Arrêt de la simulation




# --- Afficher la trajectoire des articulations ---
using Plots

# Convertir q_collect (Vector de vecteurs) en matrice (N_iterations, 7_joints)
q_matrix = hcat(q_collect...)  # crée matrice (7, N_iterations)

# Nombre d'itérations
N = size(q_matrix, 2)  # 2e dimension = nombre de colonnes (itérations)
iterations = 1:N

# Plot chaque articulation
Plots.plot(iterations, q_matrix', label=["q1" "q2" "q3" "q4" "q5" "q6" "q7"],
     xlabel="Itération", ylabel="Position (rad)", 
     title="Trajectoire des articulations", legend=:topright)

