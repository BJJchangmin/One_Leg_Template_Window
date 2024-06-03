# Compiler
CC = gcc
CXX = g++

# COMMON = /O2 /MT /EHsc /arch:AVX /I../include /Fe../bin/

# Compile options - include paths for header files
CXXFLAGS = -Wall -std=c++11 
CFLAGS = -Wall -std=c11 

# header files directory
INCDIR = -I./include -I../../include

# -IC:/myproject/mujoco/mujoco-2.2.1-windows-x86_64/include

# Library paths
LIBS = ../../lib/glfw3dll.lib ../../lib/mujoco.lib

LDFLAGS = 

HEADDIR = include
SRCDIR = src
OBJDIR = obj

# List all source files
SRCS = $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(SRCDIR)/*.c)

# Derive object file names from source file names
OBJS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(SRCS)))

# Root directory (i.e. current directory) of project
ROOT = slip_realization_kws

# Linking
all: $(OBJDIR) $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCDIR)  $(OBJS) $(LIBS) -o ../../bin/$(ROOT)

# Create the obj directory
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Compile each source file to an object file
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $< -o $@
	
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCDIR) -c $< -o $@

# $(OBJDIR)/main.o: $(HEADDIR)/array_safety.h $(HEADDIR)/controller.h $(HEADDIR)/logging.h $(HEADDIR)/uitools.h $(SRCDIR)/main.cpp
# 	$(CXX) $(CXXFLAGS) $(INCDIR) -c $(SRCDIR)/main.cpp -o $(OBJDIR)/main.o

# $(OBJDIR)/uitools.o: $(HEADDIR)/uitools.h $(SRCDIR)/uitools.c
# 	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCDIR)/uitools.c -o $(OBJDIR)/uitools.o

# $(OBJDIR)/logging.o: $(HEADDIR)/globals.h $(HEADDIR)/logging.h $(SRCDIR)/logging.c
# 	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCDIR)/logging.c  -o $(OBJDIR)/logging.o

# Clean the project
clean:
	rm -rf $(OBJDIR) ../../bin/$(ROOT)