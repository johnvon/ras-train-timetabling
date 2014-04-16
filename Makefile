CXX=		/usr/local/bin/g++-4.9

CPPFLAGS=	-I/Users/alberto/Applications/boost \
			-I/Users/alberto/Applications/cplex/include \
			-I/Users/alberto/Applications/concert/include \
			-I/Users/alberto/Desktop/TTP \
			-std=c++11 -O3 -DIL_STD -DNDEBUG -m64

LDFLAGS=	-L/Users/alberto/Applications/cplex/lib/x86-64_darwin/static_pic \
			-L/Users/alberto/Applications/concert/lib/x86-64_darwin/static_pic \
					
LDLIBS=		-lilocplex -lconcert -lcplex -lm

OBJS=		network/train.o \
			network/track.o \
			preprocessing/data.o

main: $(OBJS) main.cpp
	$(CXX) $(CPPFLAGS) $(LDFLAGS) $(OBJS) -o main $(LDLIBS) main.cpp

all: main

clean:
	rm $(OBJS)
