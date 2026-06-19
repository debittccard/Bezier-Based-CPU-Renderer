# ============================================================
#  Cross‑platform Makefile for the Tube Renderer
#
#  Requirements:
#    - Linux/macOS:   GCC or Clang (pthreads is built‑in)
#    - Windows:       MinGW‑w64  (pthreads comes with MinGW)
#                     or MSVC + vcpkg (use `cl` command directly)
# ============================================================

# ---------- Toolchain ----------
CC       ?= gcc
RM       ?= rm -f
MKDIR    ?= mkdir -p

# ---------- Target ----------
TARGET   := renderer
ifeq ($(OS),Windows_NT)
  TARGET := $(TARGET).exe
endif

# ---------- Directories (adjust if your layout differs) ----------
INC_DIRS := files/include files/src include
SRC_DIRS := files/src
BUILD_DIR := build

# ---------- Source files ----------
SRCS     := main.c scene.c renderer.c geometry.c math_utils.c
OBJS     := $(SRCS:%.c=$(BUILD_DIR)/%.o)

# ---------- Compiler flags ----------
CFLAGS   := -O3 -march=native -ffast-math -funroll-loops -pthread \
            -Wall -Wextra -std=c11
LDFLAGS  := -pthread -lm

# ---------- Cross‑platform detection ----------
ifeq ($(OS),Windows_NT)
  # MinGW already has pthreads, nothing extra needed
  # For MSVC the user should use the `cl` command shown in the help
else
  UNAME_S := $(shell uname -s)
  ifeq ($(UNAME_S),Darwin)
    # macOS – everything is built‑in
  endif
endif

# ---------- Include paths ----------
CFLAGS   += $(addprefix -I,$(INC_DIRS))

# ---------- Automatic dependency files ----------
DEPS     := $(OBJS:.o=.d)
CFLAGS   += -MMD -MP

# ---------- Default target ----------
.PHONY: all
all: check_pthread $(TARGET)

# ---------- Pre‑build check for pthreads ----------
.PHONY: check_pthread
check_pthread:
	@echo "Checking for pthreads..."
	@$(CC) -x c -pthread -o /dev/null - <<< 'int main(){return 0;}' 2>/dev/null || \
		(echo "ERROR: pthreads not found!" && \
		 echo "On Windows (MinGW): install MinGW‑w64 (pthreads is included)." && \
		 echo "On MSVC: install vcpkg and run 'vcpkg install pthreads:x64-windows'." && \
		 exit 1)
	@echo "pthreads OK."

# ---------- Linking ----------
$(TARGET): $(OBJS)
	@echo "Linking $@ ..."
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# ---------- Compilation (with auto dependency) ----------
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory if it doesn't exist
$(BUILD_DIR):
	$(MKDIR) $(BUILD_DIR)

# ---------- Include auto‑generated dependency rules ----------
-include $(DEPS)

# ---------- Clean ----------
.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR) $(TARGET)
	@echo "Cleaned."

# ---------- Help ----------
.PHONY: help
help:
	@echo "Usage:"
	@echo "  make          - build the renderer (default)"
	@echo "  make clean    - remove all build artifacts"
	@echo "  make help     - show this help"
	@echo ""
	@echo "For MSVC (Windows) without MinGW, use:"
	@echo "  cl /O2 /Fe:render.exe *.c /I files/include /I files/src /I include /link pthreads.lib"
	@echo "  (requires vcpkg with pthreads installed)"
	@echo ""
	@echo "Environment variables:"
	@echo "  CC            - compiler (default: gcc)"
	@echo "  BUILD_DIR     - where objects go (default: build)"