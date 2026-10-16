# Makefile for the myCobot 280 C++ programs.
#
#   make          build all programs (object files go in obj/)
#   make zero     build only zero
#   make clean    remove the obj/ folder
#   make fclean   remove the obj/ folder and the programs
#   make re       rebuild everything from scratch
#
# To add a program (e.g. srcs/park.cpp): add its name to PROGRAMS below.

CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -MMD -MP

SRC_DIR  := srcs
INC_DIR  := includes
OBJ_DIR  := obj

CPPFLAGS := -I$(INC_DIR)

PROGRAMS := zero joystick_all_joints joystick_display pid_tool temps_lcd
LIB_OBJS := $(OBJ_DIR)/mycobot.o $(OBJ_DIR)/joystick.o $(OBJ_DIR)/display.o $(OBJ_DIR)/lcd_display.o

.PHONY: all clean fclean re

all: $(PROGRAMS)

# Each program = its own .o + the shared class
$(PROGRAMS): %: $(OBJ_DIR)/%.o $(LIB_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile srcs/*.cpp into obj/ (the folder is created first if missing)
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(OBJ_DIR):
	mkdir -p $@

clean:
	rm -rf $(OBJ_DIR)

fclean: clean
	rm -f $(PROGRAMS)

re: fclean
	$(MAKE) --no-print-directory all

# Rebuild automatically when a header in includes/ changes
-include $(wildcard $(OBJ_DIR)/*.d)
