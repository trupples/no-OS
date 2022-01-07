CC = gcc

BINARY		= $(BUILD_DIR)/$(PROJECT_NAME).out

PLATFORM_RELATIVE_PATH = $1
PLATFORM_FULL_PATH = $1
PROJECT_BUILD = $(BUILD_DIR)/app

CFLAGS +=  -g3 \
		-DLINUX_PLATFORM \

ifeq (y,$(strip $(TINYIIOD)))
CFLAGS += -DUSE_TCP_SOCKET \
		-DENABLE_IIO_NETWORK
endif

$(PROJECT_TARGET):
	$(MUTE) $(call mk_dir, $(BUILD_DIR)) $(HIDE)
	$(MUTE) $(call set_one_time_rule,$@)

develop:
	$(MUTE) $(call mk_dir, $(PROJECT_BUILD)) $(HIDE)
	$(MUTE) $(call copy_folder, $(PLATFORM_TOOLS)/.vscode, $(PROJECT_BUILD)/.vscode) $(HIDE)
	$(MUTE) python $(PLATFORM_TOOLS)/config_build.py $(NO-OS) $(PROJECT) $(BINARY) $(HIDE)
	$(call print, Workspace ready. Open $(PROJECT_BUILD) directory in VSCode for debug)

linux_run: $(BINARY)
	$(BINARY)
