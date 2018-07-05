CC 		= gcc
FLAGS   = -lm -Ilibs -lstdc++ -pthread libs/PIDLib.a -o
WARN_AS_ERRORS_FLAGS = -pedantic-errors\
                       -Wall\
                       -Wextra\
                       -Werror\
                       -Wconversion\
                       -Wno-reorder
CPPFLAGS             = $(WARN_AS_ERRORS_FLAGS)\
                       $(FLAGS)
SRCS = example.cpp
OBJS = example.o

all: libs/PIDLib.a
	$(CC) $(SRCS) $(CPPFLAGS) $(OBJS) -std=c++11 `fltk-config --ldflags`

clean:
	 rm $(OBJS)