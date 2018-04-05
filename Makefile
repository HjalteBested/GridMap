# When clock skew appears uncomment this line
# $(shell find . -exec touch {} \;)

TARGET_EXEC := map.out

BUILD_DIR := ./build
SRC_DIRS  := ./src

CC = g++
CFLAGS = -c -Wall -O1 $(shell pkg-config --cflags opencv eigen3)

LDFLAGS = -L /usr/local/lib 
LIBS = -lpthread $(shell pkg-config --libs opencv eigen3)

SRCS := $(shell	find $(SRC_DIRS)	-name	*.cpp	-or	-name	*.c	-or	-name	*.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LIBS)

# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@ 

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR) $(TARGET_EXEC) ./Documentation	

doc:
	doxygen Doxyfile

-include $(DEPS)

MKDIR_P := mkdir -p