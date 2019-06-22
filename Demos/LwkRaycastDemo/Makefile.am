noinst_PROGRAMS=LwkRecastDemo

VehicleDemo_SOURCES=LwkRecastDemo.cpp LwkRecastDemo.h heightfield128x128.cpp main.cpp
VehicleDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
VehicleDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
