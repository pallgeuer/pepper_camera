SOURCES := $(wildcard src/*.cpp)
OBJECTS := $(patsubst %.cpp,%.o,$(SOURCES))
DEPENDS := $(patsubst %.cpp,%.d,$(SOURCES))
INCLUDE := -Iinclude

WARNING := -Wall -Wextra

.PHONY: all clean

all: pepper_camera_server

clean:
	$(RM) $(OBJECTS) $(DEPENDS) pepper_camera_server

pepper_camera_server: $(OBJECTS)
	$(CXX) $(WARNING) $(CXXFLAGS) $(INCLUDE) $^ -o $@

-include $(DEPENDS)

%.o: %.cpp Makefile
	$(CXX) $(WARNING) $(CXXFLAGS) $(INCLUDE) -MMD -MP -c $< -o $@
