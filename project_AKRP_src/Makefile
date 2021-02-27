CC = g++
#prog es el nombre del ejecutable que se creara
PROG = AKRP_exec #nombre_exec 
CFLAGS  = -Wall -std=c++11 # añadido (version compilador)

#aqui la carpeta de includes
CINCL   = -I./include # se sulen poner todos los includes en la misma carpeta
#esto hace que se compile en 2 pasos asi compila los ficheros por separado y es mas facil ver donde algo va mal
OBJS = AKRP_planner.o  C_Obstacle.o C_StaticObs.o C_Track.o C_Traj.o C_SamplePoint.o C_DynObstacle.o C_Sampling.o C_TrajGen.o C_LaneChange.o funcions_mat.o G2_Spline.o write_lane.o read_lane.o spline_vel.o Search_closest_point.o car_circles.o static_obs_function.o dynamic_obs_function.o max_admissible_vels.o sampling_dist.o check_col_sample_point.o write_vp.o write_path.o #punto .o --> ficheros objeto

$(PROG):	$(OBJS)
	$(CC) $(CINCL) -o $(PROG) $(OBJS)

#Main
AKRP_planner.o : 
	$(CC) $(CFLAGS) $(CINCL) -c AKRP_planner.cc #CINCL para buscar en una carpeta los includes

#Clases
C_Obstacle.o : C_Obstacle.h
	$(CC) $(CFLAGS) $(CINCL) -c C_Obstacle.cc
C_StaticObs.o : C_StaticObs.h
	$(CC) $(CFLAGS) $(CINCL) -c C_StaticObs.cc
C_Track.o : C_Track.h
	$(CC) $(CFLAGS) $(CINCL) -c C_Track.cc
C_Traj.o : C_Traj.h
	$(CC) $(CFLAGS) $(CINCL) -c C_Traj.cc
C_SamplePoint.o : C_SamplePoint.h
	$(CC) $(CFLAGS) $(CINCL) -c C_SamplePoint.cc
C_mandatory_point.o : C_mandatory_point.h  
	#$(CC) $(CFLAGS) $(CINCL) -c # no te fitxer .cc #potser hauria de ser una estructura...
C_DynObstacle.o : C_DynObstacle.h
	$(CC) $(CFLAGS) $(CINCL) -c C_DynObstacle.cc
C_Sampling.o : C_Sampling.h
	$(CC) $(CFLAGS) $(CINCL) -c C_Sampling.cc
C_TrajGen.o : C_TrajGen.h
	$(CC) $(CFLAGS) $(CINCL) -c C_TrajGen.cc
C_LaneChange.o : C_LaneChange.h
	$(CC) $(CFLAGS) $(CINCL) -c C_LaneChange.cc


#Estructuras
S_lane_sampling.o : S_lane_sampling.h
	#$(CC) $(CFLAGS) $(CINCL) -c C_DynObstacle.cc
S_lane_sampling_comb.o : S_lane_sampling_comb.h
	#$(CC) $(CFLAGS) $(CINCL) -c C_DynObstacle.cc


#Funciones
funcions_mat.o : funcions_mat.h
	$(CC) $(CFLAGS) $(CINCL) -c funcions_mat.cc
G2_Spline.o : G2_Spline.h
	$(CC) $(CFLAGS) $(CINCL) -c G2_Spline.cc
write_lane.o : write_lane.h
	$(CC) $(CFLAGS) $(CINCL) -c write_lane.cc
read_lane.o : read_lane.h
	$(CC) $(CFLAGS) $(CINCL) -c read_lane.cc
spline_vel.o : spline_vel.h
	$(CC) $(CFLAGS) $(CINCL) -c spline_vel.cc
Search_closest_point.o : Search_closest_point.h
	$(CC) $(CFLAGS) $(CINCL) -c Search_closest_point.cc
car_circles.o : car_circles.h
	$(CC) $(CFLAGS) $(CINCL) -c car_circles.cc
static_obs_function.o : static_obs_function.h
	$(CC) $(CFLAGS) $(CINCL) -c static_obs_function.cc
dynamic_obs_function.o : dynamic_obs_function.h
	$(CC) $(CFLAGS) $(CINCL) -c dynamic_obs_function.cc
max_admissible_vels.o : max_admissible_vels.h
	$(CC) $(CFLAGS) $(CINCL) -c max_admissible_vels.cc
sampling_dist.o : sampling_dist.h
	$(CC) $(CFLAGS) $(CINCL) -c sampling_dist.cc
check_col_sample_point.o : check_col_sample_point.h
	$(CC) $(CFLAGS) $(CINCL) -c check_col_sample_point.cc
write_vp.o : write_vp.h
	$(CC) $(CFLAGS) $(CINCL) -c write_vp.cc
write_path.o : write_path.h
	$(CC) $(CFLAGS) $(CINCL) -c write_path.cc
	
#esto borra todos los ficheros que genera el makefile asi: #make poner en el terminal / make clean --> para limpiar
# make clean
clean:
	rm -rf $(PROG) $(OBJS)

