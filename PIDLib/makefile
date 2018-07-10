CC = gcc
FLAGS      = -lm -std=c++11 -c
WARN_AS_ERRORS_FLAGS = -pedantic-errors\
                       -Wall\
                       -Wextra\
                       -Werror\
                       -Wconversion
CPPFLAGS             = $(FLAGS)\
                       $(WARN_AS_ERRORS_FLAGS)
SRCS = PIDLib.cpp FIRLowFilter.o
LIBSPATH = examples/libs
MAIN = PIDLib.a # static library
OBJS = PIDLib.o FIRLowFilter.o

all: $(MAIN)
	@echo $(MAIN) has been compiled!
	cp $(MAIN) $(LIBSPATH)
	cp PIDLib.h $(LIBSPATH)
	cp FIRLowFilter.h $(LIBSPATH)
	cp AbstractPlantModel.h $(LIBSPATH)

$(MAIN): $(OBJS)
	ar -r $(MAIN) $(OBJS)

.cpp.o:
	$(CC) $(CPPFLAGS) -c $< -o $@

clean:
	rm $(MAIN) $(OBJS)
	rm $(LIBSPATH)/$(MAIN)