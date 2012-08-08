EXEC = test
OBJS = main.o Magic_Processing.o
CXX  = g++
CXXFLAGS = -Wall -g
LIBS = -lpcl_apps -lpcl_common -lpcl_io -lpcl_kdtree -lOpenNI -lpcl_visualization -lpcl_range_image_border_extractor -lpthread -lpcl_filters -lpcl_features 
LDFLAGS = -I/usr/include/opencv -I/usr/include/openni -I/usr/include/pcl-1.5 -I/usr/include/eigen3 -I/usr/include/vtk-5.4

$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OBJS) -o $@ $(LIBS)
main.o: main.cpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -c $< $(LIBS)

Magic_Processing.o: Magic_Processing.cpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -c $< $(LIBS)



.PHONY: clean

clean:
	rm -f $(EXEC) $(OBJS) *~ 

