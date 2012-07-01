CXXFLAGS =	-g -Wall -fmessage-length=0 `pkg-config opencv --cflags`

OBJS =		OpticalFlow.o

LIBS =		`pkg-config opencv --libs`

TARGET =	OpticalFlow

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(CXXFLAGS) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
