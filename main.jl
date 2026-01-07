using LinearAlgebra
using Plots

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

# --- Constantes ---
global rob=CreateRobotKukaLwr();
global θinit=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
global θmin=(pi/180)*[-170, -120, -170, -120, -170, -120, -170];
global θmax=(pi/180)*[170, 120, 170, 120, 170, 120, 170];
global ωmax=(pi/180)*[112.5, 112.5, 112.5, 112.5, 180, 112.5, 112.5];
global Pb=[-0.3668, -0.0379, 0.5];
global Ab=RTL2R(pi/12, pi/6, 0);
global Kp=1;
global K0=1;
global dP = [0.0, 0.0, -0.001];
global dω = [0.0, 0.0, 0.0];
global dt = 0.05;

function inverseKinematicsPos(θi, Pf, rob, dP, dt)
    q = θi;
    q_collect = Vector{Vector{Float64}}()
    push!(q_collect, copy(q))
    t0 = time()
    # First time to get inside the while
    Te, MTe = MGD(q, rob);
    Pe = Te[1:3,4];
    ϵP = Pf .- Pe;
    i=0;
    while norm(ϵP) > 1e-4 && i < 1000
        # Calcul de dPe
        ϵP = Pf .- Pe;
        dPe = Kp*ϵP .+ dP;
        # Change 3 to 6 to get orientation control as well
        J = Jacobian(q, rob, Pe)[1:3, :];
        dq = LinearAlgebra.pinv(J) * (Kp*ϵP .+ dP)
        #dq = eloignement_butees_articulaires(J, dPe, q, θmax, θmin, -0.5);

        q = q + dq*dt;
        push!(q_collect, copy(q))
        #t1 = time()     
        setjointposition(clientID,q,7,0,objectname_kuka)
        t2 = time()
        #println("Temps de calcul : $(t1 - t0) secondes")
        #println("Temps VREP : $(t2 - t1) secondes")
        #println("Reste à attendre : $(dt-(t2-t0)) secondes")
        sleep(max(0.0,round(dt-(t2-t0), digits=4)))
        #println("Itération $i : Position actuelle = $Pe, Erreur = $(norm(Pf-Pe))")

        # Recalcul de Pe avec MGDrob
        t0 = time()
        Te, MTe = MGD(q, rob);
        Pe = Te[1:3,4]
        i=i+1
    end
    
    return q_collect
end

function inverseKinematics(θi, Pf, Af, rob, dP, dω, dt, λ_L=0.1)
    q = θi;
    q_collect = Vector{Vector{Float64}}()
    epsilon0_collect = Vector{Vector{Float64}}()
    push!(q_collect, copy(q))
    t0 = time()
    # First time to get inside the while
    Te, MTe = MGD(q, rob);
    Ae = Te[1:3,1:3]
    Pe = Te[1:3,4];
    ϵP = Pf .- Pe;
    ϵ0 = 0.5*(cross(Ae[1:3,1],Af[1:3,1]) + cross(Ae[1:3,2],Af[1:3,2]) + cross(Ae[1:3,3],Af[1:3,3]));
    push!(epsilon0_collect, copy(ϵ0))
    i=0;
    while (norm(ϵP) > 1e-4 || norm(ϵ0) > 1e-2) && i < 1000
        # Calcul ωe
        ϵ0 = 0.5*(cross(Ae[1:3,1],Af[1:3,1]) + cross(Ae[1:3,2],Af[1:3,2]) + cross(Ae[1:3,3],Af[1:3,3]))
        if norm(ϵ0) < 1e-2
            ωe = dω  # Pas de correction, juste suivre dω
        else
            # Calcul normal avec amortissement
            L = calcul_L(Af, Ae)
            L_damped = L' * inv(L * L' + λ_L^2 * I(3))  # Car L est mal conditionée donc pinv(L) donne des valeurs énormes
            ωe = L_damped * (K0 * ϵ0 + L' * dω)
        end
        # Calcul de dPe
        ϵP = Pf .- Pe;
        dPe = Kp*ϵP .+ dP;
        # Change 3 to 6 to get orientation control as we ll
        J = Jacobian(q, rob, Pe)[1:6, :];
        Ẋ = [dPe; ωe];
        dq = LinearAlgebra.pinv(J) * Ẋ;
        #dq = eloignement_butees_articulaires(J, Ẋ, q, θmax, θmin, -0.5);

        q = q + dq*dt;
        push!(q_collect, copy(q))
        #t1 = time()     
        setjointposition(clientID,q,7,0,objectname_kuka)
        t2 = time()
        #println("Temps de calcul : $(t1 - t0) secondes")
        #println("Temps VREP : $(t2 - t1) secondes")
        #println("Reste à attendre : $(dt-(t2-t0)) secondes")
        sleep(max(0.0,round(dt-(t2-t0), digits=4)))
        #println("Itération $i : Position actuelle = $Pe, Erreur = $(norm(Pf-Pe))")

        # Recalcul de Pe avec MGDrob
        t0 = time()
        Te, MTe = MGD(q, rob);
        Ae = Te[1:3,1:3]
        Pe = Te[1:3,4]
        ϵ0 = 0.5*(cross(Ae[1:3,1],Af[1:3,1]) + cross(Ae[1:3,2],Af[1:3,2]) + cross(Ae[1:3,3],Af[1:3,3]))
        push!(epsilon0_collect, copy(ϵ0))
        i=i+1
    end
    
    return q_collect, epsilon0_collect
end

function eloignement_butees_articulaires(J, Ẋ,  θ, θmax, θmin, α)
    n = length(θ)

    # Pseudo-inverse of Jacobian
    J_pinv = pinv(J)
    
    # Compute potential function gradient ∇φ
    θmoy = (θmax + θmin) / 2
    θrange = θmax - θmin
    
    ∇ϕ = 2 .* (θ .- θmoy) ./ (θrange .^ 2)
    
    # Main equation: θ̇ = J⁺Ẋ + (I - J⁺J)α∇φ
    theta_dot = J_pinv*Ẋ + α*(I(n) - J_pinv*J)*∇ϕ
    
    return theta_dot
end

function calcul_L(Ad, Ae)
    se_chap = chapeau(Ae[1:3,1])
    sd_chap = chapeau(Ad[1:3,1])
    ne_chap = chapeau(Ae[1:3,2])
    nd_chap = chapeau(Ad[1:3,2])
    ae_chap = chapeau(Ae[1:3,3])
    ad_chap = chapeau(Ad[1:3,3])
    L = -0.5*(se_chap*sd_chap + ne_chap*nd_chap + ae_chap*ad_chap)
    return L
end

function chapeau(v)
    v_chap = [ 0    -v[3]  v[2];
              -v[3]  0    -v[1];
              -v[2]  v[1]  0    ]
    return v_chap
end

# --- Contrôle du robot ---
# Robot en position initiale
setjointposition(clientID,θinit,7,0,objectname_kuka)
Ta, MTa = MGD(θinit, rob)
Aa = Ta[1:3,1:3]
Pa = Ta[1:3,4]
print("Robot en position : ", Pa, "\n")
sleep(2)

#q_collect = inverseKinematicsPos(θinit, Pb, rob, dP, dt)

q_collect, epsilon0_collect = inverseKinematics(θinit, Pb, Ab, rob, dP, dω, dt)

stopsimulation(clientID,simx_opmode_oneshot) # Arrêt de la simulation


#=
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
=#
# --- Afficher la trajectoire de l'effecteur ---
#=
# Récupérer la position de l'effecteur pour chaque configuration
trajectoire_effecteur = []
for q in q_collect
    pos_eff = MGD(q, rob)[1][1:3,4]
    push!(trajectoire_effecteur, pos_eff)
end

# Convertir en matrice (3, N_iterations)
trajectoire_matrix = hcat(trajectoire_effecteur...)

# Extraire x, y, z
x_traj = trajectoire_matrix[1, :]
y_traj = trajectoire_matrix[2, :]
z_traj = trajectoire_matrix[3, :]

iterations_eff = 1:length(x_traj)

# Ou plot 2D séparé pour chaque axe
Plots.plot(iterations_eff, [x_traj y_traj z_traj], 
     label=["x" "y" "z"],
     xlabel="Itération", ylabel="Position (m)", 
     title="Trajectoire de l'effecteur selon les axes", legend=:topright)
=#

# --- Afficher la trajectoire de ϵ0 (erreur d'orientation) ---
# Convertir epsilon0_collect (Vector de vecteurs) en matrice (3, N_iterations)
epsilon0_matrix = hcat(epsilon0_collect...)

# Extraire les composantes
epsilon0_x = epsilon0_matrix[1, :]
epsilon0_y = epsilon0_matrix[2, :]
epsilon0_z = epsilon0_matrix[3, :]

iterations_eps = 1:length(epsilon0_x)

# Plot des trajectoires de ϵ0
Plots.plot(iterations_eps, [epsilon0_x epsilon0_y epsilon0_z], 
     label=["ϵ0_x" "ϵ0_y" "ϵ0_z"],
     xlabel="Itération", ylabel="ϵ0 (rad)", 
     title="Trajectoire du vecteur erreur d'orientation ϵ0", legend=:topright)